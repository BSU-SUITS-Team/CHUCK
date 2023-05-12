using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
/*
public class Compass : MonoBehaviour
{
    public static Compass instance;

    [Header("assign compass parts")]
    public GameObject compass_base;
    public GameObject main_needle_prefab;
    public GameObject small_needle_prefab;

    [Serializable]
    public class PointerObject
    {
        public GameObject needle;
        public SLAPMissionObject mission_object;
        public bool is_main_target;

        public PointerObject(SLAPMissionObject point_at, bool is_main_target)
        {
            this.mission_object = point_at;
            this.is_main_target = is_main_target;
        }
    }

    [Header("debug info only")]
    public List<PointerObject> needles = new List<PointerObject>();
    public List<PointerObject> main_targets = new List<PointerObject>();


    void Awake()
    {
        instance = this;    // make singleton
    }

    void Update()
    {
        // quit out if there aren't any needles to update
        if (needles.Count < 1) return;

        // make sure there is exactly one main target
        #region selection process
        // if ambigous, promote the eligable needle with the closest target

        // if none are the main target, all needles are eligible.
        // otherwise all main targets are eligible.

        /*
        // no main targets
        if (main_targets.Count == 0)
        {
            foreach (PointerObject n in needles)
            {
                main_targets.Add(n);
            }
        }
        // too many main targets
        if (main_targets.Count > 1)
        {
            float closest_dist = 10000000000f;
            PointerObject? closest_needle = null;
            foreach (PointerObject m in main_targets)
            {
                float curr_dist = (m.needle.transform.position - m.mission_object.gameObject.transform.position).sqrMagnitude;
                if (curr_dist < closest_dist)
                {
                    closest_dist = curr_dist;
                    closest_needle = m;
                }
            }
            //PromoteNeedle(closest_needle);
        }
        
        
        #endregion

        foreach (PointerObject n in needles)
        {
            // kill any needles without a target
            if (n.mission_object == null)
            {
                needles.Remove(n);
                Destroy(n.needle);
                break;
            }
            // create any needles that don't have a gameobject but DO have a target
            if (n.needle == null)
            {
                if (!n.is_main_target) n.needle = Instantiate(small_needle_prefab, compass_base.transform);
                else n.needle = Instantiate(main_needle_prefab, compass_base.transform);
            }
        }

        // move all needles to point to their targets
        foreach (PointerObject n in needles)
        {
            n.needle.transform.rotation = Quaternion.LookRotation(
                n.needle.transform.position -
                n.mission_object.gameObject.transform.position);
        }
    }

    #region not used and broken
    public void PromoteNeedle(PointerObject needle) {
        DemoteAllNeedles();
        needle.is_main_target = true;
        Destroy(needle.needle);
        needle.needle = Instantiate(main_needle_prefab, compass_base.transform);
    }
    public void DemoteAllNeedles() {
        foreach (PointerObject n in needles)
        {
            n.is_main_target = false;
            Destroy(n.needle);
            n.needle = Instantiate(small_needle_prefab, compass_base.transform);
        }
    }
    #endregion
}
*/