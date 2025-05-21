/*
 * FirebaseBackend: abstracted functions to interact with firebase with state change callbacks
 * last updated nov 19 2024
 * for the developing dynamic application year 2.2 assignment
 *
 * copyright (c) 2024 mark joshwel
 */

using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;
using Firebase;
using Firebase.Auth;
using Firebase.Database;
using Firebase.Extensions;
using UnityEngine;

/// <summary>
///     the general managing class for handling communication with the firebase backend
///     (to be initialised by GameManager)
/// </summary>
public class Backend
{
    /// <summary>
    ///     enum for the result of the authentication process
    /// </summary>
    public enum AuthenticationResult
    {
        Ok,
        AlreadyAuthenticated,
        NonExistentUser,
        AlreadyExistingUser,
        UsernameAlreadyTaken,
        InvalidEmail,
        InvalidCredentials,
        GenericError
    }

    /// <summary>
    ///     enum for the connection status of the firebase backend
    /// </summary>
    public enum FirebaseConnectionStatus
    {
        NotConnected,
        Connected,
        UpdateRequired, // "a required system component is out of date"
        Updating, // "a required system component is updating, retrying in a bit..."
        ExternalError, // "a system component is disabled, invalid, missing, or permissions are insufficient"
        InternalError // "an unknown error occurred"
    }

    /// <summary>
    ///     generic enum for the result of a database transaction
    /// </summary>
    public enum TransactionResult
    {
        Ok,
        Unauthenticated,
        Error
    }

    public enum UserAccountDetailTargetEnum
    {
        Username,
        Email,
        Password
    }

    /// <summary>
    ///     callback functions to be invoked when the connection status changes
    /// </summary>
    /// <returns></returns>
    private readonly List<Action<FirebaseConnectionStatus>> _onConnectionStatusChangedCallbacks = new();

    /// <summary>
    ///     callback functions to be invoked when the user signs in
    /// </summary>
    private readonly List<Action<FirebaseUser>> _onSignInCallbacks = new();

    /// <summary>
    ///     callback functions to be invoked when the user signs out
    /// </summary>
    private readonly List<Action> _onSignOutCallbacks = new();

    /// <summary>
    ///     the firebase authentication object
    /// </summary>
    private FirebaseAuth _auth;

    /// <summary>
    ///     the firebase database reference
    /// </summary>
    private DatabaseReference _db;

    /// <summary>
    ///     the current user object, if authenticated
    /// </summary>
    private FirebaseUser _user;

    /// <summary>
    ///     the current user's username, if authenticated
    /// </summary>
    private string _username;

    /// <summary>
    ///     whether the user is signed in
    /// </summary>
    public bool IsSignedIn;

    /// <summary>
    ///     whether the backend is connected to the firebase backend
    /// </summary>
    public FirebaseConnectionStatus Status = FirebaseConnectionStatus.NotConnected;

    /// <summary>
    ///     variable initialisation function
    /// </summary>
    public void Initialise(Action<FirebaseConnectionStatus> callback)
    {
        FirebaseApp.CheckAndFixDependenciesAsync().ContinueWithOnMainThread(task =>
        {
            // cher is this robust enough
            switch (task.Result)
            {
                case DependencyStatus.Available:
                    _auth = FirebaseAuth.GetAuth(FirebaseApp.DefaultInstance);
                    _auth.StateChanged += AuthStateChanged;
                    _db = FirebaseDatabase.DefaultInstance.RootReference;
                    Status = FirebaseConnectionStatus.Connected;
                    callback(Status);
                    FireOnConnectionStatusChangedCallbacks();
                    break;

                case DependencyStatus.UnavailableDisabled:
                case DependencyStatus.UnavailableInvalid:
                case DependencyStatus.UnavilableMissing:
                case DependencyStatus.UnavailablePermission:
                    Status = FirebaseConnectionStatus.ExternalError;
                    callback(Status);
                    FireOnConnectionStatusChangedCallbacks();
                    break;

                case DependencyStatus.UnavailableUpdating:
                    Status = FirebaseConnectionStatus.Updating;
                    callback(Status);
                    FireOnConnectionStatusChangedCallbacks();
                    RetryInitialiseAfterDelay(callback);
                    break;

                case DependencyStatus.UnavailableUpdaterequired:
                    Status = FirebaseConnectionStatus.UpdateRequired;
                    FireOnConnectionStatusChangedCallbacks();
                    callback(Status);
                    break;

                case DependencyStatus.UnavailableOther:
                default:
                    Status = FirebaseConnectionStatus.InternalError;
                    Debug.LogError("firebase ??? blew up or something," + task.Result);
                    FireOnConnectionStatusChangedCallbacks();
                    callback(Status);
                    break;
            }

            Debug.Log("firebase status is" + Status);
        });
    }

    /// <summary>
    ///     async function to retry initialisation after a delay
    /// </summary>
    private async void RetryInitialiseAfterDelay(Action<FirebaseConnectionStatus> callback)
    {
        await Task.Delay(TimeSpan.FromSeconds(10));
        Initialise(callback);
    }

    /// <summary>
    ///     cleanup function
    /// </summary>
    public void Cleanup()
    {
        SignOutUser();
        _auth.StateChanged -= AuthStateChanged;
        _auth = null;
    }

    /// <summary>
    ///     function to register a callback for when the user signs in
    /// </summary>
    /// <param name="callback">callback function that takes in a <c>FirebaseUser</c> object</param>
    public void RegisterOnSignInCallback(Action<FirebaseUser> callback)
    {
        _onSignInCallbacks.Add(callback);
        Debug.Log($"registering OnSignInCallback ({_onSignInCallbacks.Count})");
    }

    /// <summary>
    ///     function to register a callback for when the user signs out
    /// </summary>
    /// <param name="callback">callback function</param>
    public void RegisterOnSignOutCallback(Action callback)
    {
        _onSignOutCallbacks.Add(callback);
        Debug.Log($"registering OnSignOutCallback ({_onSignOutCallbacks.Count})");
    }

    /// <summary>
    ///     function to register a callback for when the connection status changes
    /// </summary>
    /// <param name="callback">callback function that takes in a <c>FirebaseConnectionStatus</c> enum</param>
    public void RegisterOnConnectionStatusChangedCallback(Action<FirebaseConnectionStatus> callback)
    {
        _onConnectionStatusChangedCallbacks.Add(callback);
        Debug.Log($"registering ConnectionStatusChangedCallback ({_onConnectionStatusChangedCallbacks.Count})");
    }

    /// <summary>
    ///     function to fire all on sign in callbacks
    /// </summary>
    private void FireOnSignInCallbacks()
    {
        Debug.Log($"firing OnSignInCallbacks ({_onSignInCallbacks.Count})");
        foreach (var callback in _onSignInCallbacks)
            try
            {
                callback.Invoke(_user);
            }
            catch (Exception e)
            {
                Debug.LogError($"error invoking OnSignInCallback: {e.Message}");
            }
    }

    /// <summary>
    ///     function to fire all on sign-out callbacks
    /// </summary>
    private void FireOnSignOutCallbacks()
    {
        Debug.Log($"firing OnSignOutCallbacks ({_onSignOutCallbacks.Count})");
        foreach (var callback in _onSignOutCallbacks)
            try
            {
                callback.Invoke();
            }
            catch (Exception e)
            {
                Debug.LogError($"error invoking OnSignOutCallback: {e.Message}");
            }
    }

    /// <summary>
    ///     function to fire all on connection status changed callbacks
    /// </summary>
    private void FireOnConnectionStatusChangedCallbacks()
    {
        Debug.Log($"firing OnConnectionStatusChangedCallbacks ({_onConnectionStatusChangedCallbacks.Count})");
        foreach (var callback in _onConnectionStatusChangedCallbacks)
            try
            {
                callback.Invoke(Status);
            }
            catch (Exception e)
            {
                Debug.LogError($"error invoking OnConnectionStatusChangedCallback: {e.Message}");
            }
    }

    /// <summary>
    ///     function to handle the authentication state change event
    /// </summary>
    /// <param name="sender">the object that triggered the event</param>
    /// <param name="eventArgs">the event arguments</param>
    private void AuthStateChanged(object sender, EventArgs eventArgs)
    {
        // if the user hasn't changed, do nothing
        if (_auth.CurrentUser == _user) return;

        // if the user has changed, check if they've signed in or out
        IsSignedIn = _user != _auth.CurrentUser && _auth.CurrentUser != null;

        // if we're not signed in, but we still hold _user locally, we've signed out
        if (!IsSignedIn && _user != null)
        {
            Debug.Log("moi-moi");
            FireOnSignOutCallbacks();
        }

        // they have signed in, update _user
        _user = _auth.CurrentUser;
        if (!IsSignedIn) return;

        Debug.Log($"signed in successfully as {_user?.UserId}");
        RetrieveUsernameWithCallback((_, _) => { FireOnSignInCallbacks(); });
    }

    /// <summary>
    ///     abstraction function to authenticate the user
    /// </summary>
    /// <param name="email">email string</param>
    /// <param name="password">user raw password string</param>
    /// <param name="callback">callback function that takes in an <c>AuthenticationResult</c> enum</param>
    /// <param name="registerUser">whether to treat authentication as registration</param>
    /// <param name="registeringUsername">username string if registering</param>
    public void AuthenticateUser(
        string email,
        string password,
        Action<AuthenticationResult> callback,
        bool registerUser = false,
        string registeringUsername = "")
    {
        if (GameManager.Instance.Backend.GetUser() != null)
        {
            callback(AuthenticationResult.AlreadyAuthenticated);
            return;
        }

        if (registerUser)
        {
            // check if the username is already taken
            _db.Child("users")
                .OrderByChild("username")
                .EqualTo(registeringUsername)
                .GetValueAsync()
                .ContinueWithOnMainThread(task =>
                {
                    if (task.Exception != null)
                    {
                        Debug.LogError(task.Exception);
                        callback(AuthenticationResult.GenericError);
                        return;
                    }

                    if (!task.IsCompletedSuccessfully || task.Result.ChildrenCount > 0)
                    {
                        callback(AuthenticationResult.UsernameAlreadyTaken);
                        return;
                    }

                    // register user
                    _auth.CreateUserWithEmailAndPasswordAsync(email, password)
                        .ContinueWithOnMainThread(createTask =>
                        {
                            if (createTask.IsCompletedSuccessfully)
                            {
                                // store username
                                _db.Child("users")
                                    .Child(_user.UserId)
                                    .Child("username")
                                    .SetValueAsync(registeringUsername)
                                    .ContinueWithOnMainThread(setUsernameTask =>
                                    {
                                        if (setUsernameTask.IsCompletedSuccessfully)
                                        {
                                            _username = registeringUsername;
                                            callback(AuthenticationResult.Ok);
                                        }
                                        else
                                        {
                                            Debug.LogError(setUsernameTask.Exception);
                                            callback(AuthenticationResult.GenericError);
                                        }
                                    });
                                return;
                            }

                            if (createTask.Exception?.InnerException == null)
                            {
                                callback(AuthenticationResult.GenericError);
                                return;
                            }

                            var error = (AuthError)((FirebaseException)createTask.Exception.InnerException).ErrorCode;

                            // ReSharper disable once SwitchStatementHandlesSomeKnownEnumValuesWithDefault
                            switch (error)
                            {
                                case AuthError.UserNotFound:
                                    callback(AuthenticationResult.NonExistentUser);
                                    return;

                                case AuthError.InvalidEmail:
                                    callback(AuthenticationResult.InvalidEmail);
                                    return;

                                case AuthError.WeakPassword:
                                case AuthError.InvalidCredential:
                                    callback(AuthenticationResult.InvalidCredentials);
                                    return;

                                case AuthError.AccountExistsWithDifferentCredentials:
                                case AuthError.EmailAlreadyInUse:
                                    callback(AuthenticationResult.AlreadyExistingUser);
                                    return;

                                case AuthError.Failure:
                                default:
                                    Debug.LogError(error);
                                    Debug.LogError(createTask.Exception);
                                    callback(AuthenticationResult.GenericError);
                                    break;
                            }
                        });
                });
            return;
        }

        // logging in
        _auth.SignInWithEmailAndPasswordAsync(email, password)
            .ContinueWithOnMainThread(signInTask =>
            {
                if (signInTask.IsCompletedSuccessfully)
                {
                    RetrieveUsername();
                    callback(AuthenticationResult.Ok);
                    return;
                }

                if (signInTask.Exception?.InnerException == null)
                {
                    callback(AuthenticationResult.GenericError);
                    return;
                }

                var error = (AuthError)((FirebaseException)signInTask.Exception.InnerException).ErrorCode;

                // ReSharper disable once SwitchStatementHandlesSomeKnownEnumValuesWithDefault
                switch (error)
                {
                    case AuthError.UserNotFound:
                        callback(AuthenticationResult.NonExistentUser);
                        return;
                    case AuthError.InvalidEmail:
                        callback(AuthenticationResult.InvalidEmail);
                        return;
                    case AuthError.WeakPassword:
                    case AuthError.InvalidCredential:
                        callback(AuthenticationResult.InvalidCredentials);
                        return;
                    case AuthError.AccountExistsWithDifferentCredentials:
                    case AuthError.EmailAlreadyInUse:
                        callback(AuthenticationResult.AlreadyExistingUser);
                        return;
                    case AuthError.Failure:
                    default:
                        Debug.LogError(error);
                        Debug.LogError(signInTask.Exception);
                        callback(AuthenticationResult.GenericError);
                        break;
                }
            });
    }

    /// <summary>
    ///     helper function to run RetrieveUsername with no callback
    /// </summary>
    private void RetrieveUsername()
    {
        RetrieveUsernameWithCallback((_, _) => { });
    }

    /// <summary>
    ///     function to retrieve the user's username from the database
    /// </summary>
    private void RetrieveUsernameWithCallback(Action<TransactionResult, string> callback)
    {
        if (!Status.Equals(FirebaseConnectionStatus.Connected)) return;

        if (_user == null)
        {
            Debug.LogError("receiving username post-authentication but user is null (should be unreachable)");
            callback(TransactionResult.Unauthenticated, "Unknown");
            return;
        }

        _db.Child("users")
            .Child(_user.UserId)
            .Child("username")
            .GetValueAsync()
            .ContinueWithOnMainThread(task =>
            {
                TransactionResult result;
                if (task.IsCompletedSuccessfully)
                {
                    result = TransactionResult.Ok;
                    _username = task.Result.Value.ToString();
                    Debug.Log($"our username is {_username}");
                }
                else
                {
                    result = TransactionResult.Error;
                    _username = "Unknown";
                    Debug.LogError("failed to get username");
                }

                callback(result, _username);
            });
    }

    /// <summary>
    ///     abstraction function to retrieve the user
    /// </summary>
    /// <returns>the firebase user object</returns>
    public FirebaseUser GetUser()
    {
        return _user;
    }

    public string GetUsername()
    {
        return _username;
    }

    /// <summary>
    ///     abstraction function to sign out the user
    /// </summary>
    public void SignOutUser()
    {
        _auth.SignOut();
    }

    /// <summary>
    ///     abstraction function to delete the user
    /// </summary>
    public void DeleteUser(Action<TransactionResult> callback)
    {
        if (!Status.Equals(FirebaseConnectionStatus.Connected))
        {
            callback(TransactionResult.Error);
            return;
        }

        if (_user == null)
        {
            callback(TransactionResult.Unauthenticated);
            return;
        }

        _user.DeleteAsync().ContinueWithOnMainThread(task =>
        {
            if (task.IsCompletedSuccessfully)
            {
                Debug.Log("user deleted");
                _user = null;
                FireOnSignOutCallbacks();
                callback(TransactionResult.Ok);
            }
            else
            {
                Debug.LogError($"error deleting user: {task.Exception}");
                callback(TransactionResult.Error);
            }
        });
    }

    /// <summary>
    ///     abstraction function for the user to reset their password
    /// </summary>
    /// <param name="email">the forgetful user's email lol</param>
    /// <param name="callback">callback function to be invoked after the password reset email is sent</param>
    public void ResetUserPassword(string email, Action<bool> callback)
    {
        _auth.SendPasswordResetEmailAsync(email)
            .ContinueWithOnMainThread(resetTask =>
            {
                if (resetTask.IsCompletedSuccessfully)
                {
                    callback(true);
                }
                else
                {
                    Debug.LogError(resetTask.Exception);
                    callback(false);
                }
            });
    }

    /// <summary>
    ///     abstraction function to get the user's recent scores from the database
    /// </summary>
    /// <param name="callback">
    ///     callback function that takes in a <c>TransactionResult</c> enum and a
    ///     <c>List&lt;LocalPlayerData.Score&gt;</c>
    /// </param>
    public void GetRecentScores(Action<TransactionResult, List<LocalPlayerData.Score>> callback)
    {
        if (!Status.Equals(FirebaseConnectionStatus.Connected)) return;

        if (_user == null)
        {
            callback(TransactionResult.Unauthenticated, new List<LocalPlayerData.Score>(0));
            return;
        }

        // .OrderByChild("timestamp")
        //     .LimitToLast(LocalPlayerData.MaxBestScores)

        // firstly, get the user's scores
        _db.Child("scores")
            .OrderByChild("userId")
            .EqualTo(_user.UserId)
            .GetValueAsync()
            .ContinueWithOnMainThread(task =>
            {
                if (!task.IsCompletedSuccessfully)
                {
                    Debug.LogError(task.Exception);
                    callback(TransactionResult.Error, new List<LocalPlayerData.Score>(0));
                    return;
                }

                // then sort them by timestamp
                try
                {
                    var sortedScores = task.Result.Children.Select(
                            score => new LocalPlayerData.Score(score.Value as Dictionary<string, object>)
                        )
                        .OrderByDescending(score => score.Timestamp)
                        .Take(LocalPlayerData.MaxRecentScores);

                    callback(TransactionResult.Ok, sortedScores.ToList());
                    GameManager.Instance.FireLocalPlayerDataChangeCallbacks(GameManager.Instance.Data);
                }
                catch (Exception e)
                {
                    Debug.LogError($"error while sorting scores by timestamp: {e}");
                    callback(TransactionResult.Error, new List<LocalPlayerData.Score>(0));
                }
            });
    }

    /// <summary>
    ///     abstraction function to get the user's best scores from the database
    /// </summary>
    /// <param name="callback">
    ///     callback function that takes in a <c>TransactionResult</c> enum and a
    ///     <c>List&lt;LocalPlayerData.Score&gt;</c>
    /// </param>
    private void GetBestScores(Action<TransactionResult, List<LocalPlayerData.Score>> callback)
    {
        if (!Status.Equals(FirebaseConnectionStatus.Connected)) return;

        if (_user == null)
        {
            callback(TransactionResult.Unauthenticated, new List<LocalPlayerData.Score>(0));
            return;
        }

        // old code
        // _db.Child("scores")
        //     .OrderByChild("avgPerceivedAccuracy")
        //     .LimitToLast(LocalPlayerData.MaxBestScores)
        //     .GetValueAsync()
        //     .ContinueWithOnMainThread(task =>
        //     {
        //         if (!task.IsCompletedSuccessfully)
        //         {
        //             Debug.LogError(task.Exception);
        //             callback(TransactionResult.Error, new List<LocalPlayerData.Score>(0));
        //             return;
        //         }
        //
        //         var scores = new List<LocalPlayerData.Score>();
        //         foreach (var child in task.Result.Children)
        //             try
        //             {
        //                 var score = new LocalPlayerData.Score(child.Value as Dictionary<string, object>);
        //                 scores.Add(score);
        //             }
        //             catch (Exception e)
        //             {
        //                 Debug.LogError(e);
        //             }
        //
        //         callback(TransactionResult.Ok, scores);
        //         GameManager.Instance.FireLocalPlayerDataChangeCallbacks(GameManager.Instance.Data);
        //     });

        // firstly, get the user's scores
        _db.Child("scores")
            .OrderByChild("userId")
            .EqualTo(_user.UserId)
            .GetValueAsync()
            .ContinueWithOnMainThread(task =>
            {
                if (!task.IsCompletedSuccessfully)
                {
                    Debug.LogError(task.Exception);
                    callback(TransactionResult.Error, new List<LocalPlayerData.Score>(0));
                    return;
                }

                // then sort them by how good they are
                // (dL + dC + dh + de) / 4d
                try
                {
                    var sortedScores = task.Result.Children.Select(
                            score => new LocalPlayerData.Score(score.Value as Dictionary<string, object>)
                        )
                        .OrderByDescending(score =>
                            (score.AvgLightnessAccuracy + score.AvgChromaAccuracy + score.AvgHueAccuracy +
                             score.AvgPerceivedAccuracy) / 4d
                        )
                        .Take(LocalPlayerData.MaxBestScores);

                    callback(TransactionResult.Ok, sortedScores.ToList());
                    GameManager.Instance.FireLocalPlayerDataChangeCallbacks(GameManager.Instance.Data);
                }
                catch (Exception e)
                {
                    Debug.LogError($"error while sorting scores by timestamp: {e}");
                    callback(TransactionResult.Error, new List<LocalPlayerData.Score>(0));
                }
            });
    }

    /// <summary>
    ///     abstraction function to submit a score to the database
    /// </summary>
    /// <param name="score">score</param>
    /// <param name="callback">callback function that takes in a <c>TransactionResult</c> enum </param>
    public void SubmitScore(
        LocalPlayerData.Score score,
        Action<TransactionResult> callback)
    {
        if (!Status.Equals(FirebaseConnectionStatus.Connected)) return;

        if (_user == null)
        {
            callback(TransactionResult.Unauthenticated);
            return;
        }

        _db.Child("scores")
            .Push()
            .SetValueAsync(score.ToDictionary(_user.UserId))
            .ContinueWithOnMainThread(task =>
            {
                if (task.IsCompletedSuccessfully)
                {
                    callback(TransactionResult.Ok);
                }
                else
                {
                    Debug.LogError(task.Exception);
                    callback(TransactionResult.Error);
                }
            });
    }

    /// <summary>
    ///     abstraction function to get and calculate the user's rating from the database
    ///     calculation is done locally, call UpdateUserRating to update the user's rating in the database
    /// </summary>
    /// <param name="callback">
    ///     callback function that takes in a <c>TransactionResult</c> enum and a user rating
    ///     <c>float</c>
    /// </param>
    public void CalculateUserRating(
        Action<TransactionResult, float> callback)
    {
        GetRecentScores((recentRes, recentScores) =>
        {
            if (recentRes != TransactionResult.Ok)
            {
                Debug.Log("failed to get recent scores");
                callback(recentRes, 0f);
                return;
            }

            var recentScoreQueue = GameManager.Instance.Data.RecentOnlineScores;
            foreach (var score in recentScores) recentScoreQueue.Enqueue(score);
            while (recentScoreQueue.Count > LocalPlayerData.MaxRecentScores) recentScoreQueue.Dequeue();

            GetBestScores((bestRes, bestScores) =>
            {
                if (bestRes != TransactionResult.Ok)
                {
                    Debug.Log("failed to get recent scores");
                    GameManager.Instance.FireLocalPlayerDataChangeCallbacks(GameManager.Instance.Data);
                    callback(recentRes, 0f);
                    return;
                }

                var bestScoreQueue = GameManager.Instance.Data.BestOnlineScores;
                foreach (var score in bestScores) bestScoreQueue.Enqueue(score);
                while (bestScoreQueue.Count > LocalPlayerData.MaxBestScores) bestScoreQueue.Dequeue();

                GameManager.Instance.FireLocalPlayerDataChangeCallbacks(GameManager.Instance.Data);
                callback(TransactionResult.Ok, GameManager.Instance.Data.CalculateUserRating());
            });
        });
    }

    /// <summary>
    ///     abstraction function to update the user's rating in the database
    /// </summary>
    /// <param name="callback">callback function that takes in a <c>TransactionResult</c> enum </param>
    public void UpdateUserRating(
        Action<TransactionResult> callback)
    {
        if (!Status.Equals(FirebaseConnectionStatus.Connected)) return;

        if (_user == null)
        {
            callback(TransactionResult.Unauthenticated);
            return;
        }

        var userRating = GameManager.Instance.Data.CalculateUserRating();

        _db.Child("users")
            .Child(_user.UserId)
            .Child("rating")
            .SetValueAsync(userRating)
            .ContinueWithOnMainThread(task =>
            {
                if (task.IsCompletedSuccessfully)
                {
                    Debug.Log($"updated online user rating to {userRating}");
                    callback(TransactionResult.Ok);
                }
                else
                {
                    Debug.LogError(task.Exception);
                    callback(TransactionResult.Error);
                }
            });
    }

    /// <summary>
    ///     abstraction function to get the leaderboard from the database
    /// </summary>
    /// <param name="callback">
    ///     callback function that takes in a <c>TransactionResult</c> enum and a <c>List&lt;LeaderboardEntry&gt;</c>
    /// </param>
    public void GetLeaderboard(
        Action<TransactionResult, List<LeaderboardEntry>> callback)
    {
        Debug.Log("getting leaderboard");

        _db.Child("users")
            .OrderByChild("rating")
            .LimitToLast(LeaderboardUI.MaxEntries)
            .GetValueAsync()
            .ContinueWithOnMainThread(task =>
            {
                if (!task.IsCompletedSuccessfully)
                {
                    Debug.LogError(task.Exception);
                    callback(TransactionResult.Error, new List<LeaderboardEntry>(0));
                    return;
                }

                var entries = new List<LeaderboardEntry>();
                foreach (var child in task.Result.Children)
                    try
                    {
                        var entry = new LeaderboardEntry(child.Value as Dictionary<string, object>);
                        entries.Add(entry);
                    }
                    catch (Exception e)
                    {
                        Debug.LogError(e);
                    }

                callback(TransactionResult.Ok, entries);
            });
    }

    /// <summary>
    ///     abstraction function to update the user's account details in the database
    /// </summary>
    /// <param name="target">the target account detail to update</param>
    /// <param name="newValue">the new value for the target account detail</param>
    /// <param name="callback">callback function that takes in a <c>TransactionResult</c> enum</param>
    /// <exception cref="ArgumentOutOfRangeException">thrown when the target is not a valid UserAccountDetailTargetEnum</exception>
    public void UpdateUserAccountDetail(
        UserAccountDetailTargetEnum target,
        string newValue,
        Action<TransactionResult> callback)
    {
        if (!Status.Equals(FirebaseConnectionStatus.Connected)) callback(TransactionResult.Unauthenticated);

        if (_user == null)
        {
            callback(TransactionResult.Unauthenticated);
            return;
        }

        switch (target)
        {
            case UserAccountDetailTargetEnum.Email:
                _user.SendEmailVerificationBeforeUpdatingEmailAsync(newValue).ContinueWithOnMainThread(task =>
                {
                    if (task.IsCompletedSuccessfully)
                    {
                        GameManager.Instance.Data.LastKnownEmail = newValue;
                        GameManager.Instance.FireLocalPlayerDataChangeCallbacks(GameManager.Instance.Data);
                        callback(TransactionResult.Ok);
                    }
                    else
                    {
                        Debug.LogError(task.Exception);
                        callback(TransactionResult.Error);
                    }
                });
                break;

            case UserAccountDetailTargetEnum.Username:
                _db.Child("users")
                    .Child(_user.UserId)
                    .Child("username")
                    .SetValueAsync(newValue)
                    .ContinueWithOnMainThread(task =>
                    {
                        if (task.IsCompletedSuccessfully)
                        {
                            _username = newValue;
                            GameManager.Instance.Data.LastKnownUsername = newValue;
                            GameManager.Instance.FireLocalPlayerDataChangeCallbacks(GameManager.Instance.Data);
                            callback(TransactionResult.Ok);
                        }
                        else
                        {
                            Debug.LogError(task.Exception);
                            callback(TransactionResult.Error);
                        }
                    });
                break;

            case UserAccountDetailTargetEnum.Password:
                _user.UpdatePasswordAsync(newValue).ContinueWithOnMainThread(task =>
                {
                    if (task.IsCompletedSuccessfully)
                    {
                        callback(TransactionResult.Ok);
                    }
                    else
                    {
                        Debug.LogError(task.Exception);
                        callback(TransactionResult.Error);
                    }
                });
                break;

            default:
                throw new ArgumentOutOfRangeException(nameof(target), target, null);
        }
    }
}
