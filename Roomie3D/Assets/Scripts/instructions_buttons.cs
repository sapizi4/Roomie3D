// To use this example, attach this script to an empty GameObject.
// Create three buttons (Create>UI>Button). Next, select your
// empty GameObject in the Hierarchy and click and drag each of your
// Buttons from the Hierarchy to the Your First Button, Your Second Button
// and Your Third Button fields in the Inspector.
// Click each Button in Play Mode to output their message to the console.
// Note that click means press down and then release.

using UnityEngine;
using UnityEngine.UI;
using UnityEngine.SceneManagement;
using System.Collections;

public class instructions_buttons : MonoBehaviour
{
    //Make sure to attach these Buttons in the Inspector
    public Button next_button, back_button;//, m_YourThirdButton;
    public GameObject instructions1, instructions2;

    void Start()
    {
        instructions1.SetActive(true);
        instructions2.SetActive(false);
        //Calls the TaskOnClick/TaskWithParameters/ButtonClicked method when you click the Button
        next_button.onClick.AddListener(TaskOnClickNext);
        back_button.onClick.AddListener(TaskOnClickBack);
        //finish_button.onClick.AddListener(delegate { TaskWithParameters("Hello"); });
        // m_YourThirdButton.onClick.AddListener(() => ButtonClicked(42));
        //m_YourThirdButton.onClick.AddListener(TaskOnClick);
    }

    void TaskOnClickNext()
    {
        //Output this to console when Button1 is clicked
        Debug.Log("You have clicked the next button!");
        //move to second instructions
        if (instructions1.activeSelf)
        {
            instructions1.SetActive(false);
            instructions2.SetActive(true);
        //move to next scene
        } else if (instructions2.activeSelf)
        {
            //load next scene in the background
            StartCoroutine(NextScene());
         }
    }

    IEnumerator NextScene()
    {
        //move to next scene
        AsyncOperation asyncLoad = SceneManager.LoadSceneAsync("Database_1");

        // Wait until the asynchronous scene fully loads
        while (!asyncLoad.isDone)
        {
            yield return null;
        }
    }

    void TaskOnClickBack()
    {
        //Output this to console when Button1 is clicked
        Debug.Log("You have clicked the finish button!");
        instructions1.SetActive(true);
        instructions2.SetActive(false);
    }

    /*
    void TaskWithParameters(string message)
    {
        //Output this to console when the Button2 is clicked
        Debug.Log(message);
    }

    void ButtonClicked(int buttonNo)
    {
        //Output this to console when the Button3 is clicked
        Debug.Log("Button clicked = " + buttonNo);
    }
    */
}