using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

/// <summary>
/// Shared utility for notification display logic.
/// </summary>
public static class NotificationHelper
{
    /// <summary>
    /// Applies the correct color to a ColorBand Image based on severity.
    /// 0 = Red (Critical), 1 = Yellow (Warning), 2 = Purple (Info)
    /// </summary>
    public static void ApplySeverityColor(Image colorBandImage, int severity)
    {
        if (colorBandImage == null) return;

        colorBandImage.color = severity switch
        {
            0 => Color.red,
            1 => Color.yellow,
            2 => new Color(0.5f, 0f, 0.5f),
            _ => Color.white
        };
    }
}
