using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ARSIS.EventManager;

namespace ARSIS.EventManager
{
    public sealed class EventDatastore
    {
        public static EventDatastore Instance { get { return Nested.instance; } }
        private Dictionary<string, List<BaseArsisEvent>> datastore = new();
        private Dictionary<string, List<IRenderable>> handlers = new();


        public void RemoveHandler(string key, IRenderable handler)
        {
            List<IRenderable> components;
            if (handlers.TryGetValue(key, out components))
                components.Remove(handler);
        }

        public void AddHandler(string key, IRenderable handler)
        {
            List<IRenderable> components;
            if (handlers.TryGetValue(key, out components))
                components.Add(handler);
            else
                handlers.Add(key, new List<IRenderable> { handler });
        }

        private void NotifyHandlers(string key)
        {
            List<BaseArsisEvent> list;
            if (!datastore.TryGetValue(key, out list)) return;
            List<IRenderable> components;
            if (handlers.TryGetValue(key, out components))
            {
                foreach (IRenderable component in components)
                    component.Render(list);
            }
        }

        private void InitializeOrReturn(string key, out List<BaseArsisEvent> list)
        {
            if (!datastore.TryGetValue(key, out list))
            {
                list = new List<BaseArsisEvent>();
                datastore.Add(key, list);
            }
        }

        public void Append(string key, BaseArsisEvent data)
        {
            if (!datastore.ContainsKey(key))
                datastore.Add(key, new List<BaseArsisEvent>());
            List<BaseArsisEvent> list;
            InitializeOrReturn(key, out list);
            list.Add(data);
            NotifyHandlers(key);
        }

        public void Upsert(string key, BaseArsisEvent data)
        {
            List<BaseArsisEvent> upsert;
            InitializeOrReturn(key, out upsert);
            int index = upsert.FindIndex(i => i.label == data.label);
            if (index != -1) upsert[index] = data;
            else upsert.Add(data);
            NotifyHandlers(key);
        }

        private EventDatastore() {
            foreach (string key in BaseArsisEvent.EventTypes.Keys)
                datastore.Add(key, new List<BaseArsisEvent>());
        }

        private class Nested
        {
            static Nested() { }

            internal static readonly EventDatastore instance = new();
        }
    }
}
