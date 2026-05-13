using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class LowLightDimmer : MonoBehaviour
{
    [Header("References")]
    [SerializeField] private Image dimmerImage;

    [Header("Settings")]
    [SerializeField] private float lowLightAlpha = 0.25f;
    [SerializeField] private float fadeDuration = 0.5f;

    private Coroutine fadeRoutine;

    public void LowLightOn()
    {
        FadeTo(lowLightAlpha);
    }

    public void LowLightOff()
    {
        FadeTo(0f);
    }

    private void FadeTo(float targetAlpha)
    {
        if (fadeRoutine != null)
            StopCoroutine(fadeRoutine);

        fadeRoutine = StartCoroutine(FadeAlpha(targetAlpha));
    }

    private IEnumerator FadeAlpha(float targetAlpha)
    {
        float startAlpha = dimmerImage.color.a;
        float timer = 0f;

        while (timer < fadeDuration)
        {
            timer += Time.deltaTime;
            float alpha = Mathf.Lerp(startAlpha, targetAlpha, timer / fadeDuration);

            Color color = dimmerImage.color;
            color.a = alpha;
            dimmerImage.color = color;

            yield return null;
        }

        Color finalColor = dimmerImage.color;
        finalColor.a = targetAlpha;
        dimmerImage.color = finalColor;
    }
}
