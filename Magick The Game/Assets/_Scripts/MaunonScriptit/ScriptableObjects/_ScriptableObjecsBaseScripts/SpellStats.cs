using System.Collections.Generic;
using UnityEngine;
using System;

[System.Serializable]
public class SpellStats
{
    public Sprite icon;
    public string spellName;

    [Space(15)]
    public Elements.SpellElement spellElement;
    public Types.SpellTypes spellType;

    [Space(15)]
    public List<Elements.SpellElement> strongAgainstElements = new List<Elements.SpellElement>();
    public List<Elements.SpellElement> weakAgainstElements = new List<Elements.SpellElement>();
    public List<Effects> spellEffects = new List<Effects>();
    public List<Buffs> spellBuffs = new List<Buffs>();
    public List<Debuffs> spellDebuffs = new List<Debuffs>();
}
