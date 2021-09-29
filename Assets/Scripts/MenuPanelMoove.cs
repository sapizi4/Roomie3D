using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MenuPanelMoove : MonoBehaviour
{ 
    public GameObject MenuOrigPos, MenuActivePos,MenuPanel;

    public bool Move_Menu_Panel, Move_Menu_Panel_Back;
    public float MoveSpeed, tempMenuPos;

    // Start is called before the first frame update
    void Start()
    {
        //pute the menu panel to its original position
        MenuPanel.transform.position = MenuOrigPos.transform.position;
    }

    // Update is called once per frame
    void Update()
    {
        if (Move_Menu_Panel == true)
        {
            MenuPanel.transform.position = Vector3.Lerp(MenuPanel.transform.position, MenuActivePos.transform.position, MoveSpeed * Time.deltaTime);
            if(MenuPanel.transform.localPosition.x == tempMenuPos)
            {
                Move_Menu_Panel = false;
                MenuPanel.transform.position = MenuActivePos.transform.position;
                tempMenuPos = -9999999999.99f;
            }
            if(Move_Menu_Panel)
            {
                tempMenuPos = MenuPanel.transform.position.x;
            }
        }

        if(Move_Menu_Panel_Back == true)
        {
            MenuPanel.transform.position = Vector3.Lerp(MenuPanel.transform.position, MenuOrigPos.transform.position, MoveSpeed * Time.deltaTime);
            if (MenuPanel.transform.localPosition.x == tempMenuPos)
            {
                Move_Menu_Panel_Back = false;
                MenuPanel.transform.position = MenuOrigPos.transform.position;
                tempMenuPos = -9999999999.99f;
            }
            if (Move_Menu_Panel_Back)
            {
                tempMenuPos = MenuPanel.transform.position.x;
            }

        }
    }

    //when we hit the hamburger button
    public void MovePanel()
    {
        Move_Menu_Panel_Back = false;
        Move_Menu_Panel = true;
        Debug.Log("Move_Menu_Panel = " + Move_Menu_Panel + "    " + "Move_Menu_Panel_Back=" + Move_Menu_Panel_Back);
    }

    //when we hit the bacl arrow button
    public void MovePanelBack()
    {
        Move_Menu_Panel_Back = true; ;
        Move_Menu_Panel = false;
        Debug.Log("Move_Menu_Panel=" + Move_Menu_Panel + "    " + "Move_Menu_Panel_Back=" + Move_Menu_Panel_Back);
    }
}
