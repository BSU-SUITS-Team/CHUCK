using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ARSIS.EventManager;

public interface IRenderable
{
    void Render(List<BaseArsisEvent> data);
}
