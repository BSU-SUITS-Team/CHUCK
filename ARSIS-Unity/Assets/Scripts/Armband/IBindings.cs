using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ARSIS.EventManager;
using System;

public abstract class IBindings : MonoBehaviour
{
    public virtual Dictionary<KeyCode, Action> GetBindings()
    {
        return null;
    }
}
