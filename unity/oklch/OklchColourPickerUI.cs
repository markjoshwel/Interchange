/*
 * OklchColourPickerUI: demo usage of the Colorimetry script functions
 * last updated nov 17 2024
 * for the developing dynamic application year 2.2 assignment
 *
 * copyright (c) 2024 mark joshwel
 */

using System;
using UnityEngine;
using UnityEngine.UIElements;

/// <summary>
///     class to handle the oklch colour picker ui
/// </summary>
public class OklchColourPickerUI : MonoBehaviour
{
    /// <summary>
    ///     perceptual lightness value of the colour (0-100)
    /// </summary>
    public double lightness;

    /// <summary>
    ///     chroma value of the colour (0-0.5)
    /// </summary>
    public double chroma;

    /// <summary>
    ///     hue value of the colour (0-360)
    /// </summary>
    public double hue;

    /// <summary>
    ///     slider for the chroma value
    /// </summary>
    protected Slider ChromaSlider;

    /// <summary>
    ///     slider for the hue value
    /// </summary>
    protected Slider HueSlider;

    /// <summary>
    ///     slider for the lightness value
    /// </summary>
    protected Slider LightnessSlider;

    /// <summary>
    ///     visual element for the response colour preview
    /// </summary>
    protected VisualElement ResponseColour;

    /// <summary>
    ///     modify state of initialised variables
    /// </summary>
    private void Start()
    {
        LightnessSlider.value = 74.61f;
        ChromaSlider.value = 0.0868f;
        HueSlider.value = 335.72f;
    }

    /// <summary>
    ///     initialise the ui elements and register change event callbacks functions
    /// </summary>
    public void OnEnable()
    {
        var ui = GetComponent<UIDocument>().rootVisualElement;

        LightnessSlider = ui.Q<Slider>("ResponseLightnessSlider");
        LightnessSlider.RegisterCallback<ChangeEvent<float>>(OnLightnessChange);

        ChromaSlider = ui.Q<Slider>("ResponseChromaSlider");
        ChromaSlider.RegisterCallback<ChangeEvent<float>>(OnChromaChange);

        HueSlider = ui.Q<Slider>("ResponseHueSlider");
        HueSlider.RegisterCallback<ChangeEvent<float>>(OnHueChange);

        ResponseColour = ui.Q<VisualElement>("ResponseColour");
    }

    /// <summary>
    ///     handle lightness slider change
    /// </summary>
    /// <param name="evt">change event</param>
    protected void OnLightnessChange(ChangeEvent<float> evt)
    {
        lightness = Math.Clamp(evt.newValue, 0d, 100d);
        ResponseColour.style.backgroundColor = Colorimetry.RawLchToColor(lightness, chroma, hue);
    }

    /// <summary>
    ///     handle chroma slider change
    /// </summary>
    /// <param name="evt">change event</param>
    protected void OnChromaChange(ChangeEvent<float> evt)
    {
        chroma = Math.Clamp(evt.newValue, 0d, 0.5d);
        ResponseColour.style.backgroundColor = Colorimetry.RawLchToColor(lightness, chroma, hue);
    }

    /// <summary>
    ///     handle hue slider change
    /// </summary>
    /// <param name="evt">change event</param>
    protected void OnHueChange(ChangeEvent<float> evt)
    {
        hue = Math.Clamp(evt.newValue, 0d, 360d);
        ResponseColour.style.backgroundColor = Colorimetry.RawLchToColor(lightness, chroma, hue);
    }
}
