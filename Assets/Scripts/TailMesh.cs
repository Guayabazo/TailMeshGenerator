using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEditor;




[RequireComponent(typeof(MeshFilter))]
[RequireComponent(typeof(MeshRenderer))]

public class TailMesh : MonoBehaviour
{
    public enum type
    {
        targetDir,
        dampedPos,
        snake
    }
    public type movementType;
    public bool forwardTangent;
    [Range(3, 40)]
    public int length = 10;
    [Range(3, 32)]
    [SerializeField] int angularSegmentCount = 6;
    

    public AnimationCurve radiusCurve = new AnimationCurve(new Keyframe(0f, 0f), new Keyframe(0.5f, 0.5f), new Keyframe(1f, 0f));    
    

    Mesh mesh;

    private Vector3[] segmentPoses;
    private Quaternion[] rotations;
    private Vector3[] segmentV;
    private float[] radius;


    public Transform targetDir;
    public float targetDist = 0.25f;
    [Range(0, 1)]
    public float smoothSpeed = 0;
    public float trailSpeed = 10f;

    public Transform WiggleDir;
    public float wiggleSpeedX;
    public float wiggleMagnitudeX;
    
    [Range(-1, 1)]
    public float wiggleRotX = 0f;
    public float wiggleSpeedY;
    public float wiggleMagnitudeY;
    [Range(-1, 1)]
    public float wiggleRotY = 0f;
    
    public bool cosX;
    public bool cosY;


    public Transform transHelper;

    private Vector3[] forwards;
    private Vector3 lastPos;

    [Range(0, 1)]
    public float targetDamping = 0.2f;
    public float targetDistance = 0.2f;
    private Vector3 point = Vector3.zero;

    void Awake()
    {        
        rotations = new Quaternion[length];
        segmentPoses = new Vector3[length];
        for (int i = 0; i < length; i++)
            segmentPoses[i] = new Vector3(transform.position.x, transform.position.y + 0.2f, transform.position.z);
        
        segmentV = new Vector3[length];
        radius = new float[length];
        forwards = new Vector3[length];
        mesh = new Mesh();
        mesh.name = "Tail";
        GetComponent<MeshFilter>().sharedMesh = mesh;
        
        ResetPos();
        
    }
    private void OnEnable()
    {        
        rotations = new Quaternion[length];
        segmentPoses = new Vector3[length];
        for (int i = 0; i < length; i++)
            segmentPoses[i] = new Vector3(transform.position.x, transform.position.y + 0.2f, transform.position.z + (-i * targetDist));
        segmentV = new Vector3[length];
        radius = new float[length];
        forwards = new Vector3[length];
        mesh = new Mesh();
        mesh.name = "Tail";
        GetComponent<MeshFilter>().sharedMesh = mesh;
        ResetPos();
    }

    void ResetPos()
    {
        for (int i = 0; i < length; i++)
            segmentPoses[i] = new Vector3(transform.position.x, transform.position.y + 0.2f, transform.position.z + (-i * targetDist));

    }
    
    private void Update()
    {
        if (rotations == null)
            rotations = new Quaternion[length];
        if (segmentPoses == null)
            segmentPoses = new Vector3[length];
        if (segmentV == null)
            segmentV = new Vector3[length];
        if (radius == null)
            radius = new float[length];
        if (forwards == null)
            forwards = new Vector3[length];

        if (mesh == null)
        {
            mesh = new Mesh();
            mesh.name = "Tail";
            GetComponent<MeshFilter>().sharedMesh = mesh;
        }

        if (WiggleDir == null)
        {
            WiggleDir = new GameObject().transform;
            WiggleDir.name = "WiggleDir";
            WiggleDir.parent = transform;
            //links[i].localScale = Vector3.one * (1f - 1f * i / length) / parent.lossyScale.x;
            WiggleDir.localPosition = Vector3.zero;
        }

        if (targetDir == null)
        {
            targetDir = new GameObject().transform;
            targetDir.name = "TargetDir";
            targetDir.parent = WiggleDir;
            //links[i].localScale = Vector3.one * (1f - 1f * i / length) / parent.lossyScale.x;
            targetDir.localPosition = Vector3.zero;            
        }

        if (transHelper == null)
        {
            transHelper = new GameObject().transform;
            transHelper.name = "transHelper";
            transHelper.parent = WiggleDir;
            //links[i].localScale = Vector3.one * (1f - 1f * i / length) / parent.lossyScale.x;
            transHelper.localPosition = Vector3.zero;
        }

        if (rotations.Length != length)
            rotations = new Quaternion[length];
        if (segmentPoses.Length != length)
        {
            segmentPoses = new Vector3[length];
            for (int i = 0; i < length; i++)
                segmentPoses[i] = new Vector3(transform.position.x, transform.position.y + 0.1f, transform.position.z + (-i * targetDist));
        }
        if (segmentV.Length != length)
            segmentV = new Vector3[length];
        if (radius.Length != length)
            radius = new float[length];
        if (forwards.Length != length)
            forwards = new Vector3[length];


        float magnitudeX;
        float magnitudeY;
        if (!cosX)        
            magnitudeX = Mathf.Sin(Time.time * wiggleSpeedX) * wiggleMagnitudeX * wiggleRotX;                    
        else        
            magnitudeX = Mathf.Cos(Time.time * wiggleSpeedX) * wiggleMagnitudeX * wiggleRotX;        
        if (!cosY)        
            magnitudeY = Mathf.Sin(Time.time * wiggleSpeedY) * wiggleMagnitudeY * wiggleRotY;        
        else        
            magnitudeY = Mathf.Cos(Time.time * wiggleSpeedY) * wiggleMagnitudeY * wiggleRotY;
        
        //WiggleDir.localRotation = Quaternion.Euler(Mathf.Sin((Time.time + delay) * wiggleSpeed) * wiggleMagnitude, 0, 0);
        WiggleDir.localRotation = Quaternion.Euler(magnitudeX, magnitudeY, 0);
        
        segmentPoses[0] = targetDir.position;
        Vector3 tangentA = -targetDir.forward;
        //rotations[0] = Quaternion.LookRotation(tangentA, Vector3.up);

        radius[0] = 0f;
        transHelper.rotation = targetDir.rotation;
        Vector3 tangentB = -targetDir.forward;


        switch (movementType)
        {
            case type.targetDir:
                Vector3 movementDir = (transform.position - lastPos).normalized;
                point = Vector3.SmoothDamp(point, transform.position + (tangentB * targetDistance), ref segmentV[0], targetDamping);
                Handles.SphereHandleCap(0, point, Quaternion.identity, 0.1f, EventType.Repaint); //to see where I am setting the vertex
                forwards[0] = (transform.position - point).normalized;
                for (int i = 1; i < segmentPoses.Length; i++)
                {
                    radius[i] = radiusCurve.Evaluate((1f / segmentPoses.Length) * i);
                    float angleBetweenParts = Vector3.Angle(forwards[i], forwards[i - 1]);
                    Mathf.Clamp(angleBetweenParts, 0, 90);                    
                    Vector3 desiredForward = Vector3.SmoothDamp(forwards[i], forwards[i - 1], ref segmentV[i], (smoothSpeed + i / (trailSpeed + (i * 10f))) - (angleBetweenParts * 0.0002f));                    
                    forwards[i] = desiredForward;
                    segmentPoses[i] = segmentPoses[i - 1] - forwards[i] * targetDist;
                }
                break;

            case type.dampedPos:
                for (int i = 1; i < segmentPoses.Length; i++)
                {
                    radius[i] = radiusCurve.Evaluate((1f / segmentPoses.Length) * i);
                    Vector3 targetPos = (segmentPoses[i - 1] + (segmentPoses[i] - segmentPoses[i - 1]).normalized * targetDist) - targetDir.forward * 0.05f;

                    Vector3 segmentPos;
                    segmentPos = Vector3.SmoothDamp(segmentPoses[i], targetPos, ref segmentV[i],
                        smoothSpeed + i / ((trailSpeed * 100f) + (Vector3.Distance(segmentPoses[i], segmentPoses[i - 1]) - targetDist) * 300f));
                    
                    /*
                    if (Vector3.Distance(segmentPoses[i], segmentPoses[i - 1]) < Vector3.Distance(targetPos, segmentPoses[i - 1]))
                    {
                        //segmentPos = Vector3.SmoothDamp(segmentPoses[i], targetPos, ref segmentV[i], smoothSpeed + i / (trailSpeed * (Vector3.Distance(segmentPoses[i], segmentPoses[i - 1]) - targetDist) * 1000f));
                        segmentPos = targetPos;
                    }                    
                    */
                    
                    segmentPoses[i] = segmentPos;
                }

                break;


            case type.snake:
                for (int i = 1; i < segmentPoses.Length; i++)
                {
                    radius[i] = radiusCurve.Evaluate((1f / segmentPoses.Length) * i);
                    Vector3 targetPos = segmentPoses[i - 1] + (segmentPoses[i] - segmentPoses[i - 1]).normalized * targetDist;                                                                                                                                                                                                                                                          
                    segmentPoses[i] = Vector3.SmoothDamp(segmentPoses[i], targetPos, ref segmentV[i], smoothSpeed);
                }

                break;
        }


       

        for (int i = 1; i < segmentPoses.Length; i++)
        {
            
            if (segmentPoses[i] - segmentPoses[i - 1] != Vector3.zero)
            {
                tangentB = (segmentPoses[i] - segmentPoses[i - 1]).normalized;
            }

            Vector3 nextTangentB = -targetDir.TransformDirection(Vector3.forward);
            if (i < segmentPoses.Length - 1 && Vector3.Distance(segmentPoses[i+1], segmentPoses[i]) != 0)
            {
                nextTangentB = (segmentPoses[i + 1] - segmentPoses[i]).normalized;
            }
            
            //Vector3 perpendicular = new Vector3(tangentB.y, -tangentB.x, tangentB.z);


            Vector3 bisectriz = (tangentB + nextTangentB).normalized;

            //Vector3 bisectrizPerp = new Vector3(bisectriz.y, -bisectriz.x, bisectriz.z);
            //Vector3 bisectrizPerpB = new Vector3(Mathf.Abs(bisectrizPerp.x), Mathf.Abs(bisectrizPerp.y), Mathf.Abs(bisectrizPerp.z));
            Quaternion rot;
            if (!forwardTangent)
                rot = Quaternion.LookRotation(-bisectriz, targetDir.TransformDirection(Vector3.right));
            else
                rot = Quaternion.LookRotation(-bisectriz, targetDir.TransformDirection(Vector3.up));




            transHelper.rotation = rot;
            
            
            Vector3 rotA = transHelper.eulerAngles;
            Vector3 rotTargetDir = targetDir.eulerAngles;
            
            rotA = new Vector3(Mathf.Abs(rotA.x), Mathf.Abs(rotA.y), rotTargetDir.z);
            transHelper.rotation = Quaternion.Euler(rotA);


            if (Vector3.Dot(transHelper.right, targetDir.right) < 0f)
            {
                transHelper.rotation *= Quaternion.Euler(Vector3.forward * -180);
            }
            Quaternion rotC = transHelper.rotation;
            //rotations[i] = transHelper.rotation;


            float angle = Quaternion.Angle(rotC, rotations[i - 1]);
            if (angle > 90f)
                rotC *= Quaternion.Euler(Vector3.forward * -180);           
            rotations[i] = rotC;


            if (i == segmentPoses.Length - 1)
                radius[i] = 0f;            
        }

        lastPos = transform.position;
        GenerateMesh();
    }




#if UNITY_EDITOR
    public void OnDrawGizmosSelected()
    {      
        if (length > 2)
            Update();
    }

#endif

    
   

    void GenerateMesh()
    {
        if (mesh != null)
            mesh.Clear();
        List<Vector3> verts = new List<Vector3>();
        List<int> triangles = new List<int>();
        List<Vector3> normals = new List<Vector3>();
        List<Vector2> uvs = new List<Vector2>();

        for (int ring = 0; ring < length; ring++)
        {
            for (int i = 0; i < angularSegmentCount + 1; i++) //si quito el +1 de aquí y de los triangulos las normales no hacen picos, pero las uvs se fastidian
            {
                float t = i / (float)angularSegmentCount;
                float angRad = t * Mathfs.TAU;
                Vector3 dir = (Vector3)(Mathfs.GetVectorByAngle(angRad));

                Matrix4x4 worldToLocal = transform.worldToLocalMatrix;
                verts.Add(worldToLocal.MultiplyPoint3x4(((rotations[ring] * dir) * radius[ring] + segmentPoses[ring])));

                //Handles.SphereHandleCap(0, (rotations[ring] * dir) * radius[ring] + segmentPoses[ring]/* - transform.position*/, Quaternion.identity, 0.1f, EventType.Repaint); //to see where I am setting the vertex

                //Matrix4x4 localToWorld = transform.localToWorldMatrix;
                //normals.Add(localToWorld.MultiplyPoint3x4(dir));
                normals.Add(new Vector3(-dir.x, -dir.y, -dir.z));

                //tengo que duplicar los primeros vertices por las uvs //////////////
                uvs.Add(new Vector2(/*segmentPoses[ring].z*/ (1f / segmentPoses.Length) * ring, t));

                if (ring < length - 1)
                {
                    int startIndex = (angularSegmentCount + 1) * ring;
                    triangles.Add(startIndex + i + angularSegmentCount + 1);
                    triangles.Add(startIndex + (i + 1) % (angularSegmentCount + 1));
                    triangles.Add(startIndex + i);

                    triangles.Add(startIndex + i + angularSegmentCount + 1);
                    triangles.Add(startIndex + (i + 1) % (angularSegmentCount + 1) + angularSegmentCount + 1);
                    triangles.Add(startIndex + (i + 1) % (angularSegmentCount + 1));
                }
            }
        }


        mesh.SetVertices(verts);
        mesh.SetTriangles(triangles, 0);
        mesh.SetUVs(0, uvs);
        //mesh.SetNormals(normals);
        mesh.RecalculateNormals();
    }

    
}
