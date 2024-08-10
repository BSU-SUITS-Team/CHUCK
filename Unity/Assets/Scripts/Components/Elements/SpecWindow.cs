using ARSIS.EventManager;
using MixedReality.Toolkit.UX;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using TMPro;
using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.Rendering.VirtualTexturing;

public class SpecWindow : MonoBehaviour, IRenderable
{
    [SerializeField] TextMeshProUGUI rockName;
    [SerializeField] TextMeshProUGUI si;
    [SerializeField] TextMeshProUGUI ti;
    [SerializeField] TextMeshProUGUI al;
    [SerializeField] TextMeshProUGUI fe;
    [SerializeField] TextMeshProUGUI mn;
    [SerializeField] TextMeshProUGUI mg;
    [SerializeField] TextMeshProUGUI ca;
    [SerializeField] TextMeshProUGUI k;
    [SerializeField] TextMeshProUGUI p;
    [SerializeField] TextMeshProUGUI other;

    private const string FORMAT_STRING = "{1} {0:P2}";
    private List<BaseArsisEvent> data;
    private bool changed = true;
    private string key = "spec";
    private SpectrometryEva rockData = null;

    void IRenderable.Render(List<BaseArsisEvent> list)
    {
        data = list;
        RetrieveSpec(EventManager.Instance.Eva);
        changed = true;
    }

    private void RetrieveSpec(int eva)
    {
        Spectrometry spectrometry = (Spectrometry)data.LastOrDefault();
        if (spectrometry == null) {
            rockData = null;
            return;
        }
        rockData = eva switch
        {
            2 => spectrometry.data.eva2, // eva2
            _ => spectrometry.data.eva1, // default or eva1
        };
    }

    void Start()
    {
        EventDatastore eventDatastore = EventDatastore.Instance;
        eventDatastore.AddHandler(key, this);
    }

    void OnDestroy()
    {
        EventDatastore eventDatastore = EventDatastore.Instance;
        eventDatastore.RemoveHandler(key, this);
    }

    private string ElementPercent(float value, float total, string label) {
        return string.Format(FORMAT_STRING, value / total * 100, label);
    }

    // Update is called once per frame
    void Update()
    {
        if (!changed) return;
        float total = 0;
        total += rockData.data.SiO2;
        total += rockData.data.TiO2;
        total += rockData.data.Al2O3;
        total += rockData.data.FeO;
        total += rockData.data.MnO;
        total += rockData.data.MgO;
        total += rockData.data.CaO;
        total += rockData.data.K2O;
        total += rockData.data.P2O3;
        total += rockData.data.other;

        rockName.text = rockData.name;
        si.text = ElementPercent(rockData.data.SiO2, total, "SiO2");
        ti.text = ElementPercent(rockData.data.TiO2, total, "TiO2");
        al.text = ElementPercent(rockData.data.Al2O3, total, "Al2O3");
        fe.text = ElementPercent(rockData.data.FeO, total, "FeO");
        mn.text = ElementPercent(rockData.data.MnO, total, "MnO");
        mg.text = ElementPercent(rockData.data.MgO, total, "MgO");
        ca.text = ElementPercent(rockData.data.CaO, total, "CaO");
        k.text = ElementPercent(rockData.data.K2O, total, "K2O");
        p.text = ElementPercent(rockData.data.P2O3, total, "P2O3");
        other.text = ElementPercent(rockData.data.other, total, "Other");
        changed = false;
    }
}
