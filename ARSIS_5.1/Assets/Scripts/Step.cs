using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Object representing a single step in a procedure. 
/// Includes the text of the step, a picture (may be null if no picture), and an optional warning. 
/// 
/// NOTE: BestHTTP may create a line about this object that causes a compile error. 
///       This line can safely be commented out to allow compilation. 
/// </summary>
public class Step {

    private string m_task;
    private Texture2D m_picture;
    private string m_warning; 

	public Step(string task, Texture2D picture, string warning)
    {
        m_task = task;
        m_picture = picture;
        m_warning = warning; 
    }
    
    public string task
    {
        get
        {
            return m_task; 
        }
    }

    public Texture2D picture
    {
        get
        {
            return m_picture; 
        }
    }

    public string warning
    {
        get
        {
            return m_warning; 
        }
    }
}
