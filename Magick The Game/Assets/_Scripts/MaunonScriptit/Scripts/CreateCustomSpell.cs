using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class CreateCustomSpell : MonoBehaviour
{

    public SpellStats spellStats;

    #region UI references

    public InputField spellNameField;

    public Dropdown spellTypesDropdown;
    Text selectedType;
    List<string> types;

    public Dropdown spellElementDropdown;
    Text selectedElement;
    List<string> elements;

    public Button saveButton;

    #endregion

    void Start()
    {
        spellStats = new SpellStats();
        PopulateLists();
    }

    private void PopulateLists()
    {
        spellTypesDropdown.ClearOptions();
        string[] spellTypes = System.Enum.GetNames(typeof(Types.SpellTypes));
        types = new List<string>(spellTypes);
        spellTypesDropdown.AddOptions(types);

        spellElementDropdown.ClearOptions();
        string[] spellElements = System.Enum.GetNames(typeof(Elements.SpellElement));
        elements = new List<string>(spellElements);
        spellElementDropdown.AddOptions(elements);
    }

    public void DropdownIndexChanged(int index)
    {
        Types.SpellTypes spell = (Types.SpellTypes)index;
        selectedType.text = spell.ToString();

        Elements.SpellElement element = (Elements.SpellElement)index;
        selectedElement.text = element.ToString();
    }

    public void SaveSpell()
    {
        spellStats.spellName = spellNameField.text;
        spellStats.spellType = (Types.SpellTypes)spellTypesDropdown.value;
        spellStats.spellElement = (Elements.SpellElement)spellElementDropdown.value;
    }

    //void Update()
    //{
    //    if(Input.GetKeyDown(KeyCode.Space))
    //    {
    //        spellStats.spellName = "FireTornaaado";
    //        spellStats.spellType = Types.SpellTypes.Projectile;
    //    }
    //}
}
