using System.Collections;
using System.Collections.Generic;
using ARSISEventSystem;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;
using System;

public class EventSystemTests
{
    [SetUp]
    public void SetupTests()
    {
        EventManager.RemoveAllListeners();
    }

    private class TestEvent : BaseArsisEvent
    {
        public readonly int value;

        public TestEvent(int value)
        {
            this.value = value;
        }
    }

    [Test]
    public void TestAddListener()
    {
        Action<TestEvent> listener = (TestEvent e) => { };
        EventManager.AddListener<TestEvent>(listener);

        Assert.IsTrue(EventManager.IsListening<TestEvent>(listener));
    }

    [Test]
    public void TestAddMultipleListeners() {
        Action<TestEvent> listener1 = (TestEvent e) => { };
        Action<TestEvent> listener2 = (TestEvent e) => { };
        EventManager.AddListener<TestEvent>(listener1);
        EventManager.AddListener<TestEvent>(listener2);

        Assert.IsTrue(EventManager.IsListening<TestEvent>(listener1));
        Assert.IsTrue(EventManager.IsListening<TestEvent>(listener2));
    }

    [Test]
    public void TestRemoveListener() {
        Action<TestEvent> listener = (TestEvent e) => { };
        EventManager.AddListener<TestEvent>(listener);
        EventManager.RemoveListener<TestEvent>(listener);

        Assert.IsFalse(EventManager.IsListening<TestEvent>(listener));
    }

    [Test]
    public void TestRemoveMultipleListeners() {
        Action<TestEvent> listener1 = (TestEvent e) => { };
        Action<TestEvent> listener2 = (TestEvent e) => { };
        EventManager.AddListener<TestEvent>(listener1);
        EventManager.AddListener<TestEvent>(listener2);

        EventManager.RemoveListener<TestEvent>(listener1);
        EventManager.RemoveListener<TestEvent>(listener2);

        Assert.IsFalse(EventManager.IsListening<TestEvent>(listener1));
        Assert.IsFalse(EventManager.IsListening<TestEvent>(listener2));
    }

    [Test]
    public void TestTriggerEvent() {
        bool val = false;
        Action<TestEvent> listener = (TestEvent e) => { val = true; };
        EventManager.AddListener<TestEvent>(listener);

        EventManager.Trigger(new TestEvent(1));

        Assert.IsTrue(val);
    }

    [Test]
    public void TestTriggerEventMultipleTimes() {
        int val = 0;
        Action<TestEvent> listener = (TestEvent e) => { val += e.value; };
        EventManager.AddListener<TestEvent>(listener);

        EventManager.Trigger(new TestEvent(1));
        EventManager.Trigger(new TestEvent(2));
        EventManager.Trigger(new TestEvent(3));

        Assert.AreEqual(6, val);
    }

    [Test]
    public void TestTriggerMultipleEvents() {
        int val = 0;
        Action<TestEvent> singleAdd = (TestEvent e) => { val += e.value; };
        Action<TestEvent> doubleAdd = (TestEvent e) => { val += e.value * 2; };
        EventManager.AddListener<TestEvent>(singleAdd);
        EventManager.AddListener<TestEvent>(doubleAdd);

        EventManager.Trigger(new TestEvent(1));

        Assert.AreEqual(3, val);
    }

    [Test]
    public void TestTriggerMultipleEventsMultipleTimes() {
        int val = 0;
        Action<TestEvent> singleAdd = (TestEvent e) => { val += e.value; };
        Action<TestEvent> doubleAdd = (TestEvent e) => { val += e.value * 2; };
        EventManager.AddListener<TestEvent>(singleAdd);
        EventManager.AddListener<TestEvent>(doubleAdd);

        EventManager.Trigger(new TestEvent(1));
        EventManager.Trigger(new TestEvent(2));
        EventManager.Trigger(new TestEvent(3));

        Assert.AreEqual(18, val);
    }

    [Test]
    public void TestRemoveListenerDuringTrigger() {
        int val = 0;
        Action<TestEvent> listener = (TestEvent e) => { val += e.value; };
        Action<TestEvent> removeListener = (TestEvent e) => { EventManager.RemoveListener<TestEvent>(listener); };
        EventManager.AddListener<TestEvent>(listener);
        EventManager.AddListener<TestEvent>(removeListener);

        EventManager.Trigger(new TestEvent(1));
        EventManager.Trigger(new TestEvent(2));

        Assert.AreEqual(1, val);
    }

    [Test]
    public void TestAddListenerDuringTrigger() {
        int val = 0;
        Action<TestEvent> listener = (TestEvent e) => { val += e.value; };
        Action<TestEvent> addListener = (TestEvent e) =>
        {
            if (!EventManager.IsListening<TestEvent>(listener)) {
                EventManager.AddListener<TestEvent>(listener);
            }
        };
        EventManager.AddListener<TestEvent>(addListener);

        EventManager.Trigger(new TestEvent(1));
        EventManager.Trigger(new TestEvent(2));
        EventManager.Trigger(new TestEvent(3));

        Assert.AreEqual(5, val);
    }

    [Test]
    public void TestIsListening() {
        Action<TestEvent> listener = (TestEvent e) => { };
        EventManager.AddListener<TestEvent>(listener);

        Assert.IsTrue(EventManager.IsListening<TestEvent>(listener));
    }

    [Test]
    public void TestIsNotListening() {
        Action<TestEvent> listener = (TestEvent e) => { };

        Assert.IsFalse(EventManager.IsListening<TestEvent>(listener));
    }
}
