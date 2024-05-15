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
        Spectrometry spectrometry = (Spectrometry)data.Last();
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

    // Update is called once per frame
    void Update()
    {
        if (!changed) return;
        rockName.text = rockData.name;
        si.text = "SiO2 " + rockData.data.SiO2;
        ti.text = "TiO2 " + rockData.data.TiO2;
        al.text = "Al2O3 " + rockData.data.Al2O3;
        fe.text = "FeO " + rockData.data.FeO;
        mn.text = "MnO " + rockData.data.MnO;
        mg.text = "MgO " + rockData.data.MgO;
        ca.text = "CaO " + rockData.data.CaO;
        k.text = "K2O " + rockData.data.K2O;
        p.text = "P2O3 " + rockData.data.P2O3;
        other.text = "Other " + rockData.data.other;
        changed = false;
    }
}
