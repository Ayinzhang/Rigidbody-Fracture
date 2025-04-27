using UnityEditor;

[CanEditMultipleObjects] [CustomEditor(typeof(RigidbodySimulation))]
public class RigidbodySimulationEditor : Editor
{
    RigidbodySimulation m_target;
    SerializedProperty applyGravity, type;

    void OnEnable()
    {
        m_target = (RigidbodySimulation)target;
        applyGravity = serializedObject.FindProperty("applyGravity"); 
        type = serializedObject.FindProperty("type");
    }

    public override void OnInspectorGUI()
    {
        serializedObject.Update();

        EditorGUILayout.PropertyField(applyGravity);  
        if(!EditorApplication.isPlaying) EditorGUILayout.PropertyField(type);
        if ((RigidbodySimulation.SimulationType)type.enumValueIndex != 0)
            m_target.iterations = EditorGUILayout.IntSlider("Iterations", m_target.iterations, 1, 5);
        m_target.vDecay = EditorGUILayout.Slider("Velocity Decay", m_target.vDecay, 0, 1);
        if ((RigidbodySimulation.SimulationType)type.enumValueIndex == 0) 
            m_target.wDecay = EditorGUILayout.Slider("Angular Velocity Decay", m_target.wDecay, 0, 1);
        m_target.restitution = EditorGUILayout.Slider("Restitution", m_target.restitution, 0, 1);
        m_target.friction = EditorGUILayout.Slider("Friction", m_target.friction, 0, 1);

        serializedObject.ApplyModifiedProperties();
    }

}
