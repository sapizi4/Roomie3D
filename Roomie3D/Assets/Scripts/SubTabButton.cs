using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.EventSystems;
using UnityEngine.Events;


[RequireComponent(typeof(Image))]
public class SubTabButton : MonoBehaviour, IPointerEnterHandler, IPointerClickHandler, IPointerExitHandler
{

    public SubTabGroup subTabGroup;
    public Image background;
    public UnityEvent onSubTabSelected;
    public UnityEvent onSubTabDeSelected;

    public void OnPointerClick(PointerEventData eventData)
    {
        subTabGroup.OnTabSelected(this);
    }

    public void OnPointerEnter(PointerEventData eventData)
    {
        subTabGroup.OnTabEnter(this);
    }

    public void OnPointerExit(PointerEventData eventData)
    {
        subTabGroup.OnTabExit(this);
    }

    // Start is called before the first frame update
    void Start()
    {
        background = GetComponent<Image>();
        subTabGroup.Subscribe(this);

    }

    public void Select()
    {
        if (onSubTabSelected != null)
        {
            onSubTabSelected.Invoke();
        }
    }

    public void DeSelect()
    {
        if (onSubTabDeSelected != null)
        {
            onSubTabDeSelected.Invoke();
        }
    }
}

