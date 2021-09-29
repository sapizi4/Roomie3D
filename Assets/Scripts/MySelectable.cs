using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.EventSystems;
using System;

public class MySelectable : MonoBehaviour, ISelectHandler, IPointerClickHandler, IDeselectHandler
{
    public static HashSet<MySelectable> allMySelectables = new HashSet<MySelectable>();
    public static HashSet<MySelectable> currentlySelected = new HashSet<MySelectable>();

    Renderer myRenderer;

    [SerializeField]
    Material unselectedMaterial;
    [SerializeField]
    Material selectedMaterial;
    void Awake()
    {
        allMySelectables.Add(this);
        myRenderer = GetComponent<Renderer>();
    }

    public void OnDeselect(BaseEventData eventData)
    {
        myRenderer.material = unselectedMaterial;
    }

    public void OnSelect(BaseEventData eventData)
    {
        DeselectAll(eventData);
        currentlySelected.Add(this);
    }

    public void OnPointerClick(PointerEventData eventData)
    {
        OnSelect(eventData);
    }

    public static void DeselectAll(BaseEventData eventData)
    {
        foreach (MySelectable selectable in currentlySelected)
        {
            selectable.OnDeselect(eventData);
        }
        currentlySelected.Clear();
    }

}
