using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using MixedReality.Toolkit.UX;

//This script exists for a combination of two reasons:
// - ToggleCollection is part of the MRTK library, and thus doesn't get synced over GitHub
// - ToggleCollection doesn't have a version of SetSelection() that only takes one argument, and thus can't be called with UnityEvents

//I originally just edited ToggleCollection to have a single-argument version of SetSelection() before realizing it wouldn't be synced over GitHub,
//so I created this script so it would work for everyone without forcing them to edit ToggleCollection themselves

[RequireComponent(typeof(ToggleCollection))]
public class ArmbarToggleCollection : MonoBehaviour
{
    //A version of SetSelection that allows the armbar to switch between them, as UnityEvents can't call functions with more than one parameter
    public void ArmbarSetSelection(int index)
    {
        GetComponent<ToggleCollection>().SetSelection(index, true);
    }
}
