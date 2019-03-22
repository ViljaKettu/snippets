using System.Collections.Generic;
using UnityEngine;
using System;

[System.Serializable]
public struct Effects
{
    public SpellEffect effect;
    public float EffectTime;
    public float baseValue;

    public enum SpellEffect
    {
        PushBack,
        DamageOverTime,
        Heal,
        Slow
    };
}
