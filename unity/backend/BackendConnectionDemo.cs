/*
 * BackendConnectionDemo: Backend usage demo
 * copyright (c) 2025 mark joshwel
 */

using UnityEngine;

internal class DatabaseConnectionDemo : MonoBehaviour
{
    private void Start()
    {
        var b = new Backend();
        b.Init(status =>
        {
            Debug.Log("initialised backend");
            Debug.Log(
                status switch
                {
                    Backend.FirebaseConnectionStatus.Connected => "Status: Connected",
                    Backend.FirebaseConnectionStatus.Updating => "Status: Updating... (Retrying in a bit!)",
                    Backend.FirebaseConnectionStatus.NotConnected => "Status: Disconnected",
                    Backend.FirebaseConnectionStatus.UpdateRequired =>
                        "Status: Disconnected (Device Component Update Required)",
                    Backend.FirebaseConnectionStatus.ExternalError => "Status: Disconnected (External/Device Error)",
                    Backend.FirebaseConnectionStatus.InternalError => "Status: Disconnected (Internal Error)",
                    _ => "Status: Disconnected (unknown fcs state, this is unreachable and a bug)"
                }
            );

            if (status == Backend.FirebaseConnectionStatus.Connected) return;
        });

        b.RegisterOnConnectionStatusChangedCallback(status =>
        {
            Debug.Log("this is the sign in callback, this code is called when ");
            Debug.Log($"the current game-to-firebase connection status is: {status}");
        });

        // register a callback to refresh the ui when the player signs in.
        b.RegisterOnSignInCallback(user =>
        {
            Debug.Log("this is the sign in callback, this code is called when the player is signed in");
            Debug.Log($"the current user is: {user}");
        });
    }
}
