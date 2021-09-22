using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SubTabGroup : MonoBehaviour
{
    public List<SubTabButton> subTabButtons;
    public Sprite subtabIdle;
    public Sprite subtabHover;
    public Sprite subtabActive;
    public SubTabButton selectedSubTab;
    public List<GameObject> objectsToSwap;

    public void Subscribe(SubTabButton button)
    {
        if (subTabButtons == null)
        {
            subTabButtons = new List<SubTabButton>();
        }

        subTabButtons.Add(button);
    }

    public void OnTabEnter(SubTabButton button)
    {
        ResetTabs();
        if (selectedSubTab == null || button != selectedSubTab)
        {
            button.background.sprite = subtabHover;
        }

    }

    public void OnTabExit(SubTabButton button)
    {
        ResetTabs();
    }

    public void OnTabSelected(SubTabButton button)
    {
        if (selectedSubTab != null)
        {
            selectedSubTab.DeSelect();
        }

        selectedSubTab = button;
        //signing the new tab
        selectedSubTab.Select();
        ResetTabs();
        button.background.sprite = subtabActive;
        int index = button.transform.GetSiblingIndex();
        for (int i = 0; i < objectsToSwap.Count; i++)
        {
            if (i == index)
            {
                objectsToSwap[i].SetActive(true);
            }
            else
            {
                objectsToSwap[i].SetActive(false);
            }
        }
    }

    public void ResetTabs()
    {
        foreach (SubTabButton button in subTabButtons)
        {
            if (selectedSubTab != null && button == selectedSubTab) { continue; }
            button.background.sprite = subtabIdle;
        }
    }
}
