using UnityEngine;
using System;

[System.Serializable]
public struct Buffs
{
    public enum SpellBuffs
    {
        BonusDamage,
        Homing,
        FasterSpeed,
        LowerCooldown
    };

    public SpellBuffs buff;
    public float baseValue;
}
