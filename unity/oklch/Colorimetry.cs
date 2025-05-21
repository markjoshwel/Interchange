/*
 * Colorimetry: lch values to unity color script
 * last updated nov 18 2024
 * for the developing dynamic application year 2.2 assignment
 *
 * where external authorship not specified per-function,
 * this is licensed under the 0BSD licence
 *
 * copyright (c) 2020-2021 björn ottosson - mit, public domain
 * copyright (c) 2024 mark joshwel - 0bsd
 */

using System;
using UnityEngine;

/// <summary>
///     class for anything related to colour spaces and models
/// </summary>
public static class Colorimetry
{
    /// <summary>
    ///     calculate a similarity percentage from a colour distance
    /// </summary>
    /// <param name="delta">
    ///     the <c>DeltaLabChE</c> object returned by <c>CalculateDistance</c>,
    /// </param>
    /// <param name="chromaMax">
    ///     the maximum chroma value to use for the similarity percentage calculation,
    ///     defaults to 1.0f
    /// </param>
    /// <param name="hueMax">
    ///     the maximum hue value to use for the similarity percentage calculation,
    ///     defaults to 1.0f
    /// </param>
    /// <param name="lightnessMax">
    ///     the maximum lightness value to use for the similarity percentage calculation,
    ///     defaults to 1.0f
    /// </param>
    /// <returns>a <c>LCh</c> struct with 0-1f values</returns>
    public static LCh CalculateLChSimilarityPercentage(
        DeltaLabChE delta,
        double chromaMax = 1.0d,
        double hueMax = 1.0d,
        double lightnessMax = 1.0d)
    {
        // dL = [-1, 1]         lightness difference (negative = template is darker)
        // dC = [-inf, +inf]    chroma difference (negative = template is more chromatic)
        // dH = [0, +inf]       hue difference (zero for grayscale or similar hues)
        // dE = [0, 1]          overall perceptual difference in the oklab colour space
        // (but since we're using sRGB, we just use 1.0f as the max bounds)
        return new LCh((float)Math.Clamp(1 - Math.Abs(delta.dL) / lightnessMax, 0, 1),
            (float)Math.Clamp(1 - Math.Abs(delta.dC) / chromaMax, 0, 1),
            (float)Math.Clamp(1 - delta.dh / hueMax, 0, 1));
    }

    /// <summary>
    ///     calculate a 0-100% distance/accuracy between two unity rgba colour objects
    /// </summary>
    /// <param name="template">the template colour to compare against</param>
    /// <param name="response">the response colour to compare</param>
    /// <returns>a <c>DeltaLabChE</c> struct</returns>
    public static DeltaLabChE CalculateDistance(Color template, Color response)
    {
        // rgb to oklab
        var templateOklab = linear_srgb_to_oklab(new RGB(
            (float)srgb_nonlinear_transform_f_inv(template.r),
            (float)srgb_nonlinear_transform_f_inv(template.g),
            (float)srgb_nonlinear_transform_f_inv(template.b)));

        var responseOklab = linear_srgb_to_oklab(new RGB(
            (float)srgb_nonlinear_transform_f_inv(response.r),
            (float)srgb_nonlinear_transform_f_inv(response.g),
            (float)srgb_nonlinear_transform_f_inv(response.b)));

        // https://en.wikipedia.org/wiki/Oklab_color_space#Color_differences
        // ... "The perceptual color difference in Oklab is calculated as the Euclidean
        // ... distance between the (L, a, b) coordinates."
        // https://github.com/svgeesus/svgeesus.github.io/blob/master/Color/OKLab-notes.md#color-difference-metric
        // ... ΔL = L1 - L2
        // ... C1 = √(a1² + b1²)
        // ... C2 = √(a2² + b2²)
        // ... ΔC = C1 - C2
        // ... Δa = a1 - a2
        // ... Δb = b1 - b2
        // ... ΔH = √(Δa² + Δb² - ΔC²)
        // ... ΔE = √(ΔL² + ΔC² + ΔH²)

        float l1, a1, b1, l2, a2, b2;
        (l1, a1, b1) = (templateOklab.L, templateOklab.a, templateOklab.b);
        (l2, a2, b2) = (responseOklab.L, responseOklab.a, responseOklab.b);

        var deltaL = l1 - l2;
        var c1 = Math.Sqrt(a1 * a1 + b1 * b1);
        var c2 = Math.Sqrt(a2 * a2 + b2 * b2);
        var deltaC = c1 - c2;
        var deltaA = a1 - a2;
        var deltaB = b1 - b2;
        var deltaH = Math.Max(0d, Math.Sqrt(deltaA * deltaA + deltaB * deltaB - deltaC * deltaC));
        var deltaE = Math.Sqrt(deltaL * deltaL + deltaC * deltaC + deltaH * deltaH);

        return new DeltaLabChE(deltaL, deltaA, deltaB, deltaC, deltaH, deltaE);
    }

    /// <summary>
    ///     convert the oklch colour to a unity rgba colour object
    /// </summary>
    /// <returns>a unity rgba <c>Color</c> object</returns>
    public static Color RawLchToColor(double lightness, double chroma, double hue)
    {
        // clamp values
        var cL = Math.Clamp(lightness / 100.0d, 0d, 1d);
        var cC = Math.Clamp(chroma, 0d, 0.5d);
        var cH = Math.Clamp(hue, 0d, 360d);

        // convert [OKL]Ch to [OKL]ab
        var hueRadians = cH * Math.PI / 180.0d;
        var a = cC * Math.Cos(hueRadians);
        var b = cC * Math.Sin(hueRadians);

        // bring it to linear sRGB, clip it, then bring it back to non-linear sRGB
        var lsrgb = oklab_to_linear_srgb(new Lab((float)cL, (float)a, (float)b));
        var clippedLsrgb = gamut_clip_preserve_chroma(lsrgb);
        return new Color(
            Math.Clamp((float)srgb_nonlinear_transform_f(clippedLsrgb.r), 0.0f, 1.0f),
            Math.Clamp((float)srgb_nonlinear_transform_f(clippedLsrgb.g), 0.0f, 1.0f),
            Math.Clamp((float)srgb_nonlinear_transform_f(clippedLsrgb.b), 0.0f, 1.0f));
    }

    /// <summary>
    ///     transform a linear srgb value to a non-linear srgb value
    /// </summary>
    /// <param name="x">the linear srgb value to transform</param>
    /// <returns>the non-linear srgb value</returns>
    // https://bottosson.github.io/posts/colorwrong/#what-can-we-do%3F (no licence specified)
    // ReSharper disable once MemberCanBePrivate.Global
    public static double srgb_nonlinear_transform_f(double x)
    {
        if (x >= 0.0031308d)
            return 1.055d * Math.Pow(x, 1d / 2.4d) - 0.055d;
        return 12.92d * x;
    }

    /// <summary>
    ///     transform a non-linear srgb value to a linear srgb value
    /// </summary>
    /// <param name="x">the non-linear srgb value to transform</param>
    /// <returns>the linear srgb value</returns>
    // https://bottosson.github.io/posts/colorwrong/#what-can-we-do%3F (no licence specified)
    // ReSharper disable once MemberCanBePrivate.Global
    public static double srgb_nonlinear_transform_f_inv(double x)
    {
        if (x >= 0.04045d)
            return Math.Pow((x + 0.055d) / (1d + 0.055d), 2.4d);
        return x / 12.92d;
    }

    /// <summary>
    ///     clips a colour to the sRGB gamut while preserving chroma
    /// </summary>
    // https://bottosson.github.io/posts/gamutclipping/ (MIT)
    // ReSharper disable once MemberCanBePrivate.Global
    public static RGB gamut_clip_preserve_chroma(RGB rgb)
    {
        if (rgb is { r: < 1 and > 0, g: < 1 and > 0, b: < 1 and > 0 })
            return rgb;

        var lab = linear_srgb_to_oklab(rgb);

        var lchL = lab.L;
        const float eps = 0.00001f;
        var lchC = Math.Max(eps, Math.Sqrt(lab.a * lab.a + lab.b * lab.b));
        var interimA = lab.a / lchC;
        var interimB = lab.b / lchC;

        var lchL0 = Math.Clamp(lchL, 0, 1);

        var t = find_gamut_intersection((float)interimA, (float)interimB, lchL, (float)lchC, lchL0);
        var lchClippedL = lchL0 * (1 - t) + t * lchL;
        var lchClippedC = t * lchC;

        return oklab_to_linear_srgb(new Lab(lchClippedL, (float)(lchClippedC * interimA), (float)(lchClippedC *
            interimB)));
    }

    /// <summary>
    ///     Finds intersection of the line defined by
    ///     L = L0 * (1 - t) + t * L1;
    ///     C = t * C1;
    ///     a and b must be normalized so a^2 + b^2 == 1
    /// </summary>
    // https://bottosson.github.io/posts/gamutclipping/ (MIT)
    // ReSharper disable once MemberCanBePrivate.Global
    public static float find_gamut_intersection(
        float a,
        float b,
        // ReSharper disable once InconsistentNaming
        float L1,
        // ReSharper disable once InconsistentNaming
        float C1,
        // ReSharper disable once InconsistentNaming
        float L0)
    {
        // Find the cusp of the gamut triangle
        var cusp = find_cusp(a, b);

        // Find the intersection for upper and lower half separately
        float t;
        if ((L1 - L0) * cusp.C - (cusp.L - L0) * C1 <= 0f)
        {
            // Lower half
            t = cusp.C * L0 / (C1 * cusp.L + cusp.C * (L0 - L1));
        }
        else
        {
            // Upper half
            // First intersect with triangle
            t = cusp.C * (L0 - 1f) / (C1 * (cusp.L - 1f) + cusp.C * (L0 - L1));

            // Then one-step Halley's method
            {
                var dL = L1 - L0;

                var kL = +0.3963377774f * a + 0.2158037573f * b;
                var kM = -0.1055613458f * a - 0.0638541728f * b;
                var kS = -0.0894841775f * a - 1.2914855480f * b;

                // C1 = dC
                var dtL = dL + C1 * kL;
                var dtM = dL + C1 * kM;
                var dtS = dL + C1 * kS;


                // If higher accuracy is required, 2 or 3 iterations of the following block can be used:
                {
                    // ReSharper disable once InconsistentNaming
                    var L = L0 * (1f - t) + t * L1;
                    // ReSharper disable once InconsistentNaming
                    var C = t * C1;

                    var interimL = L + C * kL;
                    var interimM = L + C * kM;
                    var interimS = L + C * kS;

                    var l = interimL * interimL * interimL;
                    var m = interimM * interimM * interimM;
                    var s = interimS * interimS * interimS;

                    var ldt = 3 * dtL * interimL * interimL;
                    var mdt = 3 * dtM * interimM * interimM;
                    var sdt = 3 * dtS * interimS * interimS;

                    var ldt2 = 6 * dtL * dtL * interimL;
                    var mdt2 = 6 * dtM * dtM * interimM;
                    var sdt2 = 6 * dtS * dtS * interimS;

                    var r = 4.0767416621f * l - 3.3077115913f * m + 0.2309699292f * s - 1;
                    var r1 = 4.0767416621f * ldt - 3.3077115913f * mdt + 0.2309699292f * sdt;
                    var r2 = 4.0767416621f * ldt2 - 3.3077115913f * mdt2 + 0.2309699292f * sdt2;

                    var uR = r1 / (r1 * r1 - 0.5f * r * r2);
                    var tR = -r * uR;

                    var g = -1.2684380046f * l + 2.6097574011f * m - 0.3413193965f * s - 1;
                    var g1 = -1.2684380046f * ldt + 2.6097574011f * mdt - 0.3413193965f * sdt;
                    var g2 = -1.2684380046f * ldt2 + 2.6097574011f * mdt2 - 0.3413193965f * sdt2;

                    var uG = g1 / (g1 * g1 - 0.5f * g * g2);
                    var tG = -g * uG;

                    var newB = -0.0041960863f * l - 0.7034186147f * m + 1.7076147010f * s - 1;
                    var newB1 = -0.0041960863f * ldt - 0.7034186147f * mdt + 1.7076147010f * sdt;
                    var newB2 = -0.0041960863f * ldt2 - 0.7034186147f * mdt2 + 1.7076147010f * sdt2;

                    var uB = newB1 / (newB1 * newB1 - 0.5f * newB * newB2);
                    var tB = -newB * uB;

                    tR = uR >= 0f ? tR : float.MaxValue;
                    tG = uG >= 0f ? tG : float.MaxValue;
                    tB = uB >= 0f ? tB : float.MaxValue;

                    t += Math.Min(tR, Math.Min(tG, tB));
                }
            }
        }

        return t;
    }


    /// <summary>
    ///     finds L_cusp and C_cusp for a given hue
    ///     a and b must be normalized so a^2 + b^2 == 1
    /// </summary>
    // https://bottosson.github.io/posts/gamutclipping/ (MIT)
    // ReSharper disable once MemberCanBePrivate.Global
    public static LC find_cusp(float a, float b)
    {
        // First, find the maximum saturation (saturation S = C/L)
        var maxS = compute_max_saturation(a, b);

        // Convert to linear sRGB to find the first point where at least one of r,g or b >= 1:
        var maxedRgb = oklab_to_linear_srgb(new Lab(1, maxS * a, maxS * b));
        var cuspL = Math.Cbrt(1f / Math.Max(Math.Max(maxedRgb.r, maxedRgb.g), maxedRgb.b));
        var cuspC = cuspL * maxS;

        return new LC((float)cuspL, (float)cuspC);
    }

    // https://bottosson.github.io/posts/oklab/#converting-from-linear-srgb-to-oklab (public domain)
    // ReSharper disable once MemberCanBePrivate.Global
    public static Lab linear_srgb_to_oklab(RGB c)
    {
        var l = 0.4122214708f * c.r + 0.5363325363f * c.g + 0.0514459929f * c.b;
        var m = 0.2119034982f * c.r + 0.6806995451f * c.g + 0.1073969566f * c.b;
        var s = 0.0883024619f * c.r + 0.2817188376f * c.g + 0.6299787005f * c.b;

        var interimL = Math.Cbrt(l);
        var interimM = Math.Cbrt(m);
        var interimS = Math.Cbrt(s);

        return new Lab(
            (float)(0.2104542553f * interimL + 0.7936177850f * interimM - 0.0040720468f * interimS),
            (float)(1.9779984951f * interimL - 2.4285922050f * interimM + 0.4505937099f * interimS),
            (float)(0.0259040371f * interimL + 0.7827717662f * interimM - 0.8086757660f * interimS)
        );
    }

    // https://bottosson.github.io/posts/oklab/#converting-from-linear-srgb-to-oklab (public domain)
    // ReSharper disable once MemberCanBePrivate.Global
    public static RGB oklab_to_linear_srgb(Lab c)
    {
        var interimL = c.L + 0.3963377774f * c.a + 0.2158037573f * c.b;
        var interimM = c.L - 0.1055613458f * c.a - 0.0638541728f * c.b;
        var interimS = c.L - 0.0894841775f * c.a - 1.2914855480f * c.b;

        var l = interimL * interimL * interimL;
        var m = interimM * interimM * interimM;
        var s = interimS * interimS * interimS;

        return new RGB(
            +4.0767416621f * l - 3.3077115913f * m + 0.2309699292f * s,
            -1.2684380046f * l + 2.6097574011f * m - 0.3413193965f * s,
            -0.0041960863f * l - 0.7034186147f * m + 1.7076147010f * s
        );
    }

    /// <summary>
    ///     Finds the maximum saturation possible for a given hue that fits in sRGB
    ///     Saturation here is defined as S = C/L
    ///     a and b must be normalized so a^2 + b^2 == 1
    /// </summary>
    // https://bottosson.github.io/posts/gamutclipping/ (MIT)
    // ReSharper disable once MemberCanBePrivate.Global
    public static float compute_max_saturation(float a, float b)
    {
        // Max saturation will be when one of r, g or b goes below zero.

        // Select different coefficients depending on which component goes below zero first
        float k0, k1, k2, k3, k4, wl, wm, ws;

        if (-1.88170328f * a - 0.80936493f * b > 1)
        {
            // Red component
            k0 = +1.19086277f;
            k1 = +1.76576728f;
            k2 = +0.59662641f;
            k3 = +0.75515197f;
            k4 = +0.56771245f;
            wl = +4.0767416621f;
            wm = -3.3077115913f;
            ws = +0.2309699292f;
        }
        else if (1.81444104f * a - 1.19445276f * b > 1)
        {
            // Green component
            k0 = +0.73956515f;
            k1 = -0.45954404f;
            k2 = +0.08285427f;
            k3 = +0.12541070f;
            k4 = +0.14503204f;
            wl = -1.2684380046f;
            wm = +2.6097574011f;
            ws = -0.3413193965f;
        }
        else
        {
            // Blue component
            k0 = +1.35733652f;
            k1 = -0.00915799f;
            k2 = -1.15130210f;
            k3 = -0.50559606f;
            k4 = +0.00692167f;
            wl = -0.0041960863f;
            wm = -0.7034186147f;
            ws = +1.7076147010f;
        }

        // Approximate max saturation using a polynomial:
        var maxSaturation = k0 + k1 * a + k2 * b + k3 * a * a + k4 * a * b;

        // Do one-step Halley's method to get closer
        // this gives an error less than 10e6, except for some blue hues where the dS/dh is close to infinite
        // this should be enough for most applications, otherwise do two/three steps 

        var kL = +0.3963377774f * a + 0.2158037573f * b;
        var kM = -0.1055613458f * a - 0.0638541728f * b;
        var kS = -0.0894841775f * a - 1.2914855480f * b;

        {
            var interimL = 1f + maxSaturation * kL;
            var interimM = 1f + maxSaturation * kM;
            var interimS = 1f + maxSaturation * kS;

            var l = interimL * interimL * interimL;
            var m = interimM * interimM * interimM;
            var s = interimS * interimS * interimS;

            var sDerivL = 3f * kL * interimL * interimL;
            var sDerivM = 3f * kM * interimM * interimM;
            var sDerivS = 3f * kS * interimS * interimS;

            var sDeriv2L = 6f * kL * kL * interimL;
            var sDeriv2M = 6f * kM * kM * interimM;
            var sDeriv2S = 6f * kS * kS * interimS;

            var f = wl * l + wm * m + ws * s;
            var f1 = wl * sDerivL + wm * sDerivM + ws * sDerivS;
            var f2 = wl * sDeriv2L + wm * sDeriv2M + ws * sDeriv2S;

            maxSaturation -= f * f1 / (f1 * f1 - 0.5f * f * f2);
        }

        return maxSaturation;
    }

    // ReSharper disable once InconsistentNaming
    public struct DeltaLabChE
    {
        // ReSharper disable once InconsistentNaming
        public readonly double dL;

        // ReSharper disable once InconsistentNaming
        public readonly double da;

        // ReSharper disable once InconsistentNaming
        public readonly double db;

        // ReSharper disable once InconsistentNaming
        public readonly double dC;

        // ReSharper disable once InconsistentNaming
        public readonly double dh;

        // ReSharper disable once InconsistentNaming
        public readonly double dE;

        public DeltaLabChE(
            // ReSharper disable once InconsistentNaming
            double L,
            double a,
            double b,
            // ReSharper disable once InconsistentNaming
            double C,
            // ReSharper disable once InconsistentNaming
            double H,
            // ReSharper disable once InconsistentNaming
            double E)
        {
            dL = L;
            da = a;
            db = b;
            dC = C;
            dh = H;
            dE = E;
        }
    }

    public readonly struct Lab
    {
        public readonly float L;

        // ReSharper disable once InconsistentNaming
        public readonly float a;

        // ReSharper disable once InconsistentNaming
        public readonly float b;

        // ReSharper disable once InconsistentNaming
        public Lab(float L, float a, float b)
        {
            this.L = L;
            this.a = a;
            this.b = b;
        }
    }

    public readonly struct LCh
    {
        public readonly float L;
        public readonly float C;

        // ReSharper disable once InconsistentNaming
        public readonly float h;

        public LCh(
            // ReSharper disable once InconsistentNaming
            float L,
            // ReSharper disable once InconsistentNaming
            float C,
            float h)
        {
            this.L = L;
            this.C = C;
            this.h = h;
        }
    }

    public readonly struct RGB
    {
        // ReSharper disable once InconsistentNaming
        public readonly float r;

        // ReSharper disable once InconsistentNaming
        public readonly float g;

        // ReSharper disable once InconsistentNaming
        public readonly float b;

        public RGB(float r, float g, float b)
        {
            this.r = r;
            this.g = g;
            this.b = b;
        }
    }

    // ReSharper disable once InconsistentNaming
    public readonly struct LC
    {
        public readonly float L;
        public readonly float C;

        public LC(
            // ReSharper disable once InconsistentNaming
            float L,
            // ReSharper disable once InconsistentNaming
            float C)
        {
            this.L = L;
            this.C = C;
        }
    }
}
