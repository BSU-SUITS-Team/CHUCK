// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

using System;
using System.Runtime.InteropServices;
using System.Globalization;
using UnityEngine;
using UnityEngine.XR.Management;

#if UNITY_EDITOR
using UnityEditor;
using UnityEditor.XR.OpenXR.Features;
#endif

namespace Microsoft.MixedReality.OpenXR.Remoting
{
#if UNITY_EDITOR
    [OpenXRFeature(UiName = featureName,
        BuildTargetGroups = new[] { BuildTargetGroup.Standalone, BuildTargetGroup.WSA },
        Company = "Microsoft",
        Desc = "Feature to enable " + featureName + ".",
        DocumentationLink = "https://aka.ms/openxr-unity-app-remoting",
        OpenxrExtensionStrings = requestedExtensions,
        Category = FeatureCategory.Feature,
        Required = false,
        Priority = -100,    // hookup before other plugins so it affects json before GetProcAddr.
        FeatureId = featureId,
        Version = "1.5.0")]
#endif
    [NativeLibToken(NativeLibToken = NativeLibToken.Remoting)]
    internal class AppRemotingPlugin : OpenXRFeaturePlugin<AppRemotingPlugin>
    {
        private enum RemotingState
        {
            Idle = 0,
            Connect = 1,
            Listen = 2,
            Disconnecting = 3
        }

        private static RemotingState  s_remotingState;
        private RemotingConfiguration m_remotingConfiguration;
        private RemotingListenConfiguration m_remotingListenConfiguration;
        private SecureConnectionServerConfiguration m_secureConnectionServerConfiguration;

        internal const string featureId = "com.microsoft.openxr.feature.appremoting";
        internal const string featureName = "Holographic Remoting remote app";
        private const string requestedExtensions = "XR_MSFT_holographic_remoting XR_MSFT_holographic_remoting_speech";

        private bool m_runtimeOverrideAttempted = false;
        private OpenXRRuntimeRestartHandler m_restartHandler = null;

        private readonly bool m_appRemotingIsActive =
#if UNITY_EDITOR
            false;
#else
            true;
#endif

        protected override IntPtr HookGetInstanceProcAddr(IntPtr func)
        {
            if (m_appRemotingIsActive)
            {
                if(NativeLib.IsHighIntegrityLevel(nativeLibToken))
                {
                    Debug.LogError($"Because the application is running in high integrity level, the Holographic Remoting OpenXR runtime cannot be located by Mixed Reality OpenXR plugin package.");
                }
                else
                {
                    if (!m_runtimeOverrideAttempted)
                    {
                        m_runtimeOverrideAttempted = true;
                        if (!NativeLib.TryEnableRemotingOverride(nativeLibToken))
                        {
                            Debug.LogError($"Failed to enable remoting runtime.");
                        }
                    }
                    return NativeLib.HookGetInstanceProcAddr(nativeLibToken, func);
                }
            }
            return func;
        }

        private void Awake()
        {
            if (m_appRemotingIsActive && enabled && m_restartHandler == null)
            {
                m_restartHandler = new OpenXRRuntimeRestartHandler(this, skipRestart: true, skipQuitApp: true);
            }
        }

        private void OnDestroy()
        {
            if (m_restartHandler != null)
            {
                m_restartHandler.Dispose();
                m_restartHandler = null;
            }
        }

        protected override void OnInstanceDestroy(ulong instance)
        {
            if (m_appRemotingIsActive && m_runtimeOverrideAttempted)
            {
                m_runtimeOverrideAttempted = false;
                NativeLib.ResetRemotingOverride(nativeLibToken);
            }

            Debug.Log($"[AppRemotingPlugin] OnInstanceDestroy, remotingState was {s_remotingState}.");
            if (s_remotingState != RemotingState.Listen)
            {
                s_remotingState = RemotingState.Idle;
            }
            base.OnInstanceDestroy(instance);
        }

        protected override void OnSystemChange(ulong systemId)
        {
            base.OnSystemChange(systemId);

            if (systemId != 0 && m_appRemotingIsActive)
            {
                Debug.Log($"[AppRemotingPlugin] OnSystemChange, systemId = {systemId}, remotingState = {s_remotingState}.");
                NativeLib.SetRemoteSpeechCulture(nativeLibToken, CultureInfo.CurrentCulture.Name);

                if (s_remotingState == RemotingState.Connect)
                {
                    NativeLib.ConnectRemoting(nativeLibToken, m_remotingConfiguration);
                }
                else if (s_remotingState == RemotingState.Listen)
                {
                    NativeLib.ListenRemoting(nativeLibToken, m_remotingListenConfiguration, m_secureConnectionServerConfiguration.SecureListen,
                        m_secureConnectionServerConfiguration.CertificateStorePath, m_secureConnectionServerConfiguration.SubjectName,
                        m_secureConnectionServerConfiguration.KeyPassphrase, m_secureConnectionServerConfiguration.AuthenticationRealm,
                        m_secureConnectionServerConfiguration.Token);
                }
            }
        }

        protected override void OnSessionStateChange(int oldState, int newState)
        {
            if (m_appRemotingIsActive && (XrSessionState)newState == XrSessionState.LossPending)
            {
                if (s_remotingState == RemotingState.Connect)
                {
                    Debug.LogError($"[AppRemotingPlugin] Cannot establish a connection to Holographic Remoting Player " +
                        $"on the target with IP Address {m_remotingConfiguration.RemoteHostName}:{m_remotingConfiguration.RemotePort}.");
                }
                else if (s_remotingState == RemotingState.Listen)
                {
                    Debug.Log("[AppRemotingPlugin] Listening to incoming Holographic Remoting connection is interrupted.");
                }
            }
        }

        public System.Collections.IEnumerator Connect(RemotingConfiguration configuration)
        {
            if (s_remotingState == RemotingState.Idle)
            {
                m_remotingConfiguration = configuration;
                m_remotingListenConfiguration = default;
                s_remotingState = RemotingState.Connect;

                if (XRGeneralSettings.Instance.Manager.activeLoader == null)
                {
                    Debug.Log("[AppRemotingPlugin] Connect InitializeLoader");
                    yield return XRGeneralSettings.Instance.Manager.InitializeLoader();
                }

                if (XRGeneralSettings.Instance.Manager.activeLoader != null)
                {
                    Debug.Log("[AppRemotingPlugin] Connect StartSubsystems");
                    XRGeneralSettings.Instance.Manager.StartSubsystems();
                }
            }
            else
            {
                Debug.LogError("Cannot connect when previous connection is still in progress");
            }
        }

        public System.Collections.IEnumerator Listen(RemotingListenConfiguration configuration, SecureConnectionServerConfiguration secureConnectionServerConfiguration, Action onRemotingListenCompleted = null)
        {
            var defaultWait = new WaitForSeconds(0.5f);

            if (s_remotingState == RemotingState.Idle)
            {
                m_remotingListenConfiguration = configuration;
                m_remotingConfiguration = default;
                m_secureConnectionServerConfiguration = secureConnectionServerConfiguration;
                s_remotingState = RemotingState.Listen;

                while (s_remotingState == RemotingState.Listen)
                {
                    if (XRGeneralSettings.Instance.Manager.activeLoader == null)
                    {
                        Debug.Log("[AppRemotingPlugin] Listen, InitializeLoader");
                        yield return XRGeneralSettings.Instance.Manager.InitializeLoader();
                    }

                    if (XRGeneralSettings.Instance.Manager.activeLoader != null)
                    {
                        Debug.Log("[AppRemotingPlugin] Listen, StartSubsystems");
                        XRGeneralSettings.Instance.Manager.StartSubsystems();
                        yield return defaultWait;
                    }

                    while (true)
                    {
                        if (!TryGetConnectionState(out ConnectionState connectionState, out _) ||
                            connectionState == ConnectionState.Disconnected)
                        {
                            Debug.Log("[AppRemotingPlugin] Listen, After disconnection, Stop XR Loader.");
                            StopXrLoader();
                            break;  // If disconnected, stop XR session and try to restart.
                        }
                        if (XRGeneralSettings.Instance.Manager.activeLoader == null)
                        {
                            break;  // if XR loader is already stopped, try to restart.
                        }

                        yield return defaultWait;
                    }

                    Debug.Log("[AppRemotingPlugin] Listen, Try restart XR session");
                    yield return defaultWait;
                }
            }
            else
            {
                Debug.LogError("[AppRemotingPlugin] Cannot listen when previous connection is still in progress");
            }

            if (onRemotingListenCompleted != null)
            {
                onRemotingListenCompleted.Invoke();
            }
        }

        private class AppRemotingDisconnectHelper : MonoBehaviour
        {   
            private void Start()
            {
                StartCoroutine(NotifySubsystemDestroyAndDisconnect());
            }
            public static System.Collections.IEnumerator NotifySubsystemDestroyAndDisconnect()
            {
                if (OpenXRContext.Current.Instance != 0)
                {
                    // Notify the AR Foundation subsystems before the subsystem destroy and 
                    // allow some time for cleaning up
                    NativeLib.DestroyAnchorSubsystemPending(NativeLibToken.HoloLens);

                    // wait for one frame to make sure the Anchor changes are notified to Unity on GetAnchorChanges() callback
                    yield return null;

                    NativeLib.RemoveAllAnchors(NativeLibToken.HoloLens);

                    // wait for one frame to make sure removed anchors are notified
                    yield return null;

                    NativeLib.DisconnectRemoting(NativeLibToken.Remoting);
                }

                AppRemotingPlugin.StopXrLoader();
                AppRemotingPlugin.s_remotingState = RemotingState.Idle;
            }
        }

        public void Disconnect()
        {
            if(s_remotingState != RemotingState.Disconnecting)
            {
                s_remotingState = RemotingState.Disconnecting;
                _ = new GameObject("AppRemotingDisconnectHelper", typeof(AppRemotingDisconnectHelper))
                {
                    hideFlags = HideFlags.HideAndDontSave
                };
            }
        }

        private static void StopXrLoader()
        {
            if (XRGeneralSettings.Instance.Manager.activeLoader != null)
            {
                XRGeneralSettings.Instance.Manager.StopSubsystems();
                Debug.Log("[AppRemotingPlugin] Disconnect StopSubsystems");

                if (XRGeneralSettings.Instance.Manager.isInitializationComplete)
                {
                    XRGeneralSettings.Instance.Manager.DeinitializeLoader();
                    Debug.Log("[AppRemotingPlugin] Disconnect DeinitializeLoader");
                }
            }
        }

        public bool TryGetConnectionState(out ConnectionState connectionState, out DisconnectReason disconnectReason)
        {
            return NativeLib.TryGetRemotingConnectionState(NativeLibToken.Remoting, out connectionState, out disconnectReason);
        }

        public bool TryLocateUserReferenceSpace(FrameTime frameTime, out Pose pose)
        {
            return NativeLib.TryLocateUserReferenceSpace(NativeLibToken.Remoting, frameTime, out pose);
        }

#if UNITY_EDITOR
        protected override void GetValidationChecks(System.Collections.Generic.List<ValidationRule> results, BuildTargetGroup targetGroup)
        {
            AppRemotingValidator.GetValidationChecks(this, results, targetGroup);
        }
#endif
    }

    [Serializable, StructLayout(LayoutKind.Sequential, Pack = 8)]
    internal struct SecureConnectionServerConfiguration
    {
        // ignore other fields when SecureListen is false
        public bool SecureListen;
        public string CertificateStorePath;
        public string SubjectName;
        public string KeyPassphrase;
        public string AuthenticationRealm;
        public string Token;
    }
}
