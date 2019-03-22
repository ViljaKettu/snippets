using UnityEngine;
using System;

[System.Serializable]
public struct Debuffs
{
    public enum SpellDebuffs
    {
        LongerCooldown,
        SlowerSpeed,
        DamageCastingPlayer,
        ChangeToFail,
        LimitedUses
    };

    public SpellDebuffs debuff;
    public float baseValue;
}
