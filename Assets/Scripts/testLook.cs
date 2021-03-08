using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class testLook : MonoBehaviour
{
    public Transform frontObj, upObj;
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        Vector3 frontDir = frontObj.position - this.transform.position;
        Vector3 upDir = upObj.position - this.transform.position;

        Quaternion quat = Quaternion.LookRotation(frontDir, upDir);

        this.transform.rotation = quat;
    }
}
