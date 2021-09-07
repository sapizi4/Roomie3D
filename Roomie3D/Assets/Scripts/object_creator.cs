using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class object_creator : MonoBehaviour
{
    private int currentObject;
    [SerializeField] private Button nextButton;
    [SerializeField] private Button prevButton;


    private void Awake()
    {
        SelectObject(0);
    }
    private void SelectObject(int _index)
    { 
        prevButton.interactable = (_index != 0);
        nextButton.interactable = (_index != transform.childCount - 1);
        for (int i = 0; i < transform.childCount; i++)
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
