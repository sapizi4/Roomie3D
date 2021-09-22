using Dummiesman;
using System.IO;
using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using UnityEngine.UI;

[ExecuteInEditMode]
public class upload_room : MonoBehaviour
{
    [SerializeField] private Button room;
    [SerializeField] private InputField objPathInput;
    string objPath = string.Empty;
    string error = string.Empty;
    GameObject actual_room;

    public void clickLoadRoom()
    {
        objPath = objPathInput.text;
        //file path
        if (!File.Exists(objPath))
        {
            error = "File doesn't exist.";
        }
        else
        {
            if (actual_room != null)
                Destroy(actual_room);
            actual_room = new OBJLoader().Load(objPath);
            error = string.Empty;
        }
    }
}
