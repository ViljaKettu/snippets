using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public struct Floors 
{
    string name;
    int floorId;
    float sizeX, sizeY;

    public Floors(string _name, int _floorId, float _sizeX, float _sizeY)
    {
        name = _name;
        floorId = _floorId;
        sizeX = _sizeX;
        sizeY = _sizeY;
    }
}
