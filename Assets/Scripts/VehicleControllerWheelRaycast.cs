using System.Collections;
using System.Collections.Generic;
using UnityEngine;

// https://www.youtube.com/watch?v=CdPYlj5uZeI
public class VehicleControllerWheelRaycast : MonoBehaviour
{    
    [SerializeField]
    Rigidbody carRigidBody;
    [SerializeField]
    Transform carTransform;

    [Header("Suspension")]
    [SerializeField]
    float wheelRadius;
    [SerializeField]
    float suspensionRestDist;
    [SerializeField]
    float springStrength;
    [SerializeField]
    float springDamper;

    [Header("Steering and Grip")]
    [SerializeField, Tooltip("0=no grip, 1=full grip")]
    AnimationCurve tireGripFactor;
    [SerializeField]
    float tireMass;
    [SerializeField]
    float maxSteeringAngle;
    [SerializeField]
    bool isSteerable;
    [SerializeField]
    Transform tireTransform;
    [SerializeField]
    Transform tireGraphics;

    [Header("Engine")]
    [SerializeField]
    AnimationCurve enginePower;
    [SerializeField]
    float carTopSpeed;
    [SerializeField]
    float maxEngineTorque;

    [Header("Wheel Friction")]
    [SerializeField]
    float wheelFriction;

    [Header("Braking")]
    [SerializeField, Tooltip("0=no brake, 1=full brake"), Range(0f,1f)]
    float brakeStrength;

    bool breakInput;
    float accelInput;
    float wheelDistancePerRev, wheelAngularVel;


    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        if (isSteerable)
        {
            float steeringInput = Input.GetAxis("Horizontal");
            tireTransform.right = Quaternion.AngleAxis(steeringInput * maxSteeringAngle, carTransform.up) * carTransform.right;
        }
        accelInput = Input.GetAxis("Vertical");
        breakInput = Input.GetKey(KeyCode.Space);
    }

    private void FixedUpdate()
    {
        RaycastHit hit;
        Physics.Raycast(transform.position, -transform.up, out hit, suspensionRestDist + wheelRadius);
        if (hit.collider != null) {
            Debug.DrawLine(transform.position, transform.position - transform.up * (hit.distance), Color.yellow);
            Vector3 springDir = tireTransform.up;
            Vector3 tireWorldVel = carRigidBody.GetPointVelocity(tireTransform.position);
            float offset = suspensionRestDist - hit.distance;
            Debug.DrawLine(transform.position, transform.position - transform.up * (offset), Color.red);
            float vel = Vector3.Dot(springDir, tireWorldVel);
            float force = (offset * springStrength) - (vel * springDamper);
            carRigidBody.AddForceAtPosition(springDir * force, tireTransform.position);
            tireTransform.position = transform.position + (-hit.distance + wheelRadius) * transform.up;


            // Steering
            Debug.DrawRay(tireTransform.position, tireTransform.forward, Color.blue);
            Debug.DrawRay(tireTransform.position, tireTransform.right, Color.red);
            Vector3 steeringDir = tireTransform.right;
            float steeringVel = Vector3.Dot(steeringDir, tireWorldVel);
            float desiredVelChange = -steeringVel * tireGripFactor.Evaluate(steeringVel);
            float desiredAccel = desiredVelChange / Time.fixedDeltaTime;
            carRigidBody.AddForceAtPosition(steeringDir * tireMass * desiredAccel, tireTransform.position);

            // Engine
            Vector3 accelDir = tireTransform.forward;
            float carSpeed = Vector3.Dot(carTransform.forward, carRigidBody.velocity);
            if (Mathf.Abs(accelInput) > 0f)
            {
                // forward speed of car in direction of driving
                float normalizedSpeed = Mathf.Clamp01(Mathf.Abs(carSpeed) / carTopSpeed);
                float availableTorque = enginePower.Evaluate(normalizedSpeed) * accelInput * maxEngineTorque;
                carRigidBody.AddForceAtPosition(accelDir * availableTorque, tireTransform.position);
            }
            wheelDistancePerRev = 180f * wheelRadius;
            wheelAngularVel = tireWorldVel.magnitude * (1 / wheelDistancePerRev);
            print(wheelAngularVel * 360f);
            tireGraphics.Rotate(Vector3.right * transform.localScale.x * carSpeed, wheelAngularVel * 360f, Space.Self);

            // Wheel friction
            Vector3 wheelFrictionDir = tireTransform.forward;
            float wheelFrictionVel = Vector3.Dot(wheelFrictionDir, tireWorldVel);
            float desiredFrictionVelChange = -wheelFrictionVel * wheelFriction;
            float desiredWheelFrictionAccel = desiredFrictionVelChange / Time.fixedDeltaTime;
            carRigidBody.AddForceAtPosition(wheelFrictionDir * tireMass * desiredWheelFrictionAccel, tireTransform.position);

            // Brakes
            Vector3 breakingDir = tireTransform.forward;
            float breakingVel = Vector3.Dot(breakingDir, tireWorldVel);
            float desiredBrakeVelChange = -breakingVel * brakeStrength;
            float desiredBrakeAccel = desiredBrakeVelChange / Time.fixedDeltaTime;
            carRigidBody.AddForceAtPosition(breakingDir * tireMass * desiredBrakeAccel, tireTransform.position);
        }
    }
}
