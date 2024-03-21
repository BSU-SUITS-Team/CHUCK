using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ARSIS.EventManager;

public interface IRenderable
{
    public void Render(List<BaseArsisEvent> data);
}
