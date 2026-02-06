using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ARSIS.EventManager;

public interface IRenderable
{
    /**
     * Gets executed within a callback by the WebSocketClient.
     * Cannot invoke Instantiate() or Destroy() in a separate thread.
     */
    void Render(List<BaseArsisEvent> data);
}
