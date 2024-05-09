using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace ARSIS.EventManager
{
    [System.Serializable]
    public class RockData
    {
        public float SiO2 { get; set; }
        public float TiO2 { get; set; }
        public float Al2O3 { get; set; }
        public float FeO { get; set; }
        public float MnO { get; set; }
        public float MgO { get; set; }
        public float CaO { get; set; }
        public float K2O { get; set; }
        public float P2O3 { get; set; }
        public float other { get; set; }
    }

    [System.Serializable]
    public class SpectrometryEva
    {
        public RockData data { get; set; }
        public int id { get; set; }
        public string name { get; set; }
    }

    [System.Serializable]
    public class SpectrometryData
    {
        public SpectrometryEva eva1 { get; set; }
        public SpectrometryEva eva2 { get; set; }
    }

    [System.Serializable]
    public class Spectrometry : BaseArsisEvent
    {
        public SpectrometryData data { get; set; }

        public override string ToString()
        {
            return string.Join(
                "eva1 {",
                "\tSiO2: " + data.eva1.data.SiO2,
                "\tTiO2: " + data.eva1.data.TiO2,
                "\tAl2O3: " + data.eva1.data.Al2O3,
                "\tFeO: " + data.eva1.data.FeO,
                "\tMnO: " + data.eva1.data.MnO,
                "\tMgO: " + data.eva1.data.MgO,
                "\tCaO: " + data.eva1.data.CaO,
                "\tK2O: " + data.eva1.data.K2O,
                "\tP2O3: " + data.eva1.data.P2O3,
                "\tother: " + data.eva1.data.other,
                "},",
                "eva2 {",
                "\tSiO2: " + data.eva2.data.SiO2,
                "\tTiO2: " + data.eva2.data.TiO2,
                "\tAl2O3: " + data.eva2.data.Al2O3,
                "\tFeO: " + data.eva2.data.FeO,
                "\tMnO: " + data.eva2.data.MnO,
                "\tMgO: " + data.eva2.data.MgO,
                "\tCaO: " + data.eva2.data.CaO,
                "\tK2O: " + data.eva2.data.K2O,
                "\tP2O3: " + data.eva2.data.P2O3,
                "\tother: " + data.eva2.data.other,
                "}"
                , Environment.NewLine);
        }
    }
}
