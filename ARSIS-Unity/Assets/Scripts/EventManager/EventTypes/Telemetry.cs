using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace ARSIS.EventManager
{
	[System.Serializable]
	public class Eva
	{
		public float batt_time_left;
		public float oxy_pri_storage;
		public float oxy_sec_storage;
		public float oxy_pri_pressure;
		public float oxy_sec_pressure;
		public float oxy_time_left;
		public float heart_rate;
		public float oxy_consumption;
		public float co2_production;
		public float suit_pressure_oxy;
		public float suit_pressure_co2;
		public float suit_pressure_other;
		public float suit_pressure_total;
		public float fan_pri_rpm;
		public float fan_sec_rpm;
		public float helmet_pressure_co2;
		public float scrubber_a_co2_storage;
		public float scrubber_b_co2_storage;
		public float temperature;
		public float coolant_ml;
		public float coolant_gas_pressure;
		public float coolant_liquid_pressure;
	}

	[System.Serializable]
	public class Data
	{
		public int eva_time;
		public Eva eva1;
		public Eva eva2;
	}

	[System.Serializable]
	public class Root
	{
		public Data telemetry;
	}

	[System.Serializable]
    public class Telemetry : BaseArsisEvent
    {
		public static string Type = "telemetry";
		public Root data;

		public Telemetry() : base()
		{
			type = Type;
		}

        public override string ToString()
        {
			return "Telemetry event";
        }
    }
}
