using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class add_object_to_scene : MonoBehaviour
{
    private int currentObject;
    [SerializeField] private Button add_Button;

    private void Awake()
    {
        SelectObject(0);
    }
    private void SelectObject(int _index)
    {
        for (int i=0; i < transform.childCount; i++)
        {
            transform.GetChild(i).gameObject.SetActive(i == _index);
        }
    }
    public void ChangeObject(int _change)
    {
        currentObject += _change;
        SelectObject(currentObject);
    }
}