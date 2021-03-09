using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using System.IO;

public class main : MonoBehaviour
{
    /// <summary> Joints Name
    /// rShldrBend 0, rForearmBend 1, rHand 2, rThumb2 3, rMid1 4,
    /// lShldrBend 5, lForearmBend 6, lHand 7, lThumb2 8, lMid1 9,
    /// lEar 10, lEye 11, rEar 12, rEye 13, Nose 14,
    /// rThighBend 15, rShin 16, rFoot 17, rToe 18,
    /// lThighBend 19, lShin 20, lFoot 21, lToe 22,    
    /// abdomenUpper 23,
    /// hip 24, head 25, neck 26, spine 27
    /// </summary>
    // Start is called before the first frame update
    public Transform indicator;

    // threeDpose项目中AI预测的关键点坐标及其相关信息
    int[] parent = new int[] { 26, 0, 1, 2, 2, 26, 5, 6, 7, 7, 25, 10, 11, 12, 13, 24, 15, 16, 17, 24, 19, 20, 21, 24, -1, 26, 27, 23 }; // 28个关节
    private List<List<Vector3>> pose3D = new List<List<Vector3>>();
    private float tallHeadNeck, tallNeckSpine, tallSpineCrotch, tallThigh, tallShin;
    private float prevTall = 448 * 0.75f;
    private float centerTall = 448 * 0.75f;
    private float movementScale = 0.01f * 224f / 448;
    private Vector3 initPos;
    //动画相关
    Animator animator;
    private Transform root, spine, neck, head, leye, reye, lshoulder, lelbow, lhand, lthumb2, lmid1, rshoulder, relbow, rhand, rthumb2, rmid1, lhip, lknee, lfoot, ltoe, rhip, rknee, rfoot, rtoe;
    private Quaternion midRoot, midSpine, midNeck, midHead, midLshoulder, midLelbow, midLhand, midRshoulder, midRelbow, midRhand, midLhip, midLknee, midLfoot, midRhip, midRknee, midRfoot;
    public Transform nose;
    // 播放相关
    private int frame = 0;
    private int playRatio = 20;
    void Start()
    {
        // 读取关键点数据
        StreamReader sr = new StreamReader(@"D:\code\Unity\ThreeDExperiment\Assets\Resources\record.txt", System.Text.Encoding.Default);
        string line;
        while ((line = sr.ReadLine()) != null)
        {
            float[] tmp = new float[84];
            int count = 0;
            foreach (string num in line.Split(' '))
            {
                tmp[count] = float.Parse(num);
                count = count + 1;
                if (count >= 84)
                {
                    break;
                }
            }
            List<Vector3> posvec = new List<Vector3>();
            for(int i = 0; i < 28; i++)
            {
                posvec.Add(new Vector3(tmp[i * 3], tmp[i * 3 + 1], tmp[i * 3 + 2]));
            }
            pose3D.Add(posvec);
        }

        // 动画相关
        animator = this.GetComponent<Animator>();
        /////////////////////////////////////////////////// 骨骼定义 ///////////////////////////////////////////////////
        //躯干
        root = animator.GetBoneTransform(HumanBodyBones.Hips);
        spine = animator.GetBoneTransform(HumanBodyBones.Spine);
        neck = animator.GetBoneTransform(HumanBodyBones.Neck);
        head = animator.GetBoneTransform(HumanBodyBones.Head);
        leye = animator.GetBoneTransform(HumanBodyBones.LeftEye);
        reye = animator.GetBoneTransform(HumanBodyBones.RightEye);
        //左臂
        lshoulder = animator.GetBoneTransform(HumanBodyBones.LeftUpperArm);
        lelbow = animator.GetBoneTransform(HumanBodyBones.LeftLowerArm);
        lhand = animator.GetBoneTransform(HumanBodyBones.LeftHand);
        lthumb2 = animator.GetBoneTransform(HumanBodyBones.LeftThumbIntermediate);
        lmid1 = animator.GetBoneTransform(HumanBodyBones.LeftMiddleProximal);
        //右臂
        rshoulder = animator.GetBoneTransform(HumanBodyBones.RightUpperArm);
        relbow = animator.GetBoneTransform(HumanBodyBones.RightLowerArm);
        rhand = animator.GetBoneTransform(HumanBodyBones.RightHand);
        rthumb2 = animator.GetBoneTransform(HumanBodyBones.RightThumbIntermediate);
        rmid1 = animator.GetBoneTransform(HumanBodyBones.RightMiddleDistal);
        //左腿
        lhip = animator.GetBoneTransform(HumanBodyBones.LeftUpperLeg);
        lknee = animator.GetBoneTransform(HumanBodyBones.LeftLowerLeg);
        lfoot = animator.GetBoneTransform(HumanBodyBones.LeftFoot);
        ltoe = animator.GetBoneTransform(HumanBodyBones.LeftToes);
        //右腿
        rhip = animator.GetBoneTransform(HumanBodyBones.RightUpperLeg);
        rknee = animator.GetBoneTransform(HumanBodyBones.RightLowerLeg);
        rfoot = animator.GetBoneTransform(HumanBodyBones.RightFoot);
        rtoe = animator.GetBoneTransform(HumanBodyBones.RightToes);

        initPos = root.position;
        /////////////////////////////////////////////////// 骨骼中间变换矩阵 ///////////////////////////////////////////////////
        // 当前旋转 = lookforward * 中间矩阵
        // 对于初始姿态，当前旋转就是初始旋转，结合骨骼方向和人体方向，求解各关节的中间矩阵
        Vector3 forward = TriangleNormal(root.position, lhip.position, rhip.position);
        // midLshoulder, midLelbow, midLhand, midRscapular, midRshoulder, midRelbow, midRhand, midLhip, midLknee, midLfoot, midLtoe, midRhip, midRknee, midRfoot, midRtoe        
        // Root
        midRoot = Quaternion.Inverse(root.rotation) * Quaternion.LookRotation(forward);
        // 躯干
        midSpine = Quaternion.Inverse(spine.rotation) * Quaternion.LookRotation(spine.position - neck.position, forward);
        midNeck = Quaternion.Inverse(neck.rotation) * Quaternion.LookRotation(neck.position - head.position, forward);
        // 头部
        midHead = Quaternion.Inverse(head.rotation) * Quaternion.LookRotation(nose.position-head.position);
        // 左臂
        midLshoulder = Quaternion.Inverse(lshoulder.rotation) * Quaternion.LookRotation(lshoulder.position - lelbow.position, forward);
        midLelbow = Quaternion.Inverse(lelbow.rotation) * Quaternion.LookRotation(lelbow.position - lhand.position, forward);
        midLhand = Quaternion.Inverse(lhand.rotation) * Quaternion.LookRotation(
            lthumb2.position - lmid1.position,
            TriangleNormal(lhand.position, lthumb2.position, lmid1.position)
            );
        // 右臂
        midRshoulder = Quaternion.Inverse(rshoulder.rotation) * Quaternion.LookRotation(rshoulder.position - relbow.position, forward);
        midRelbow = Quaternion.Inverse(relbow.rotation) * Quaternion.LookRotation(relbow.position - rhand.position, forward);
        midRhand = Quaternion.Inverse(rhand.rotation) * Quaternion.LookRotation(
            rthumb2.position - rmid1.position,
            TriangleNormal(rhand.position, rthumb2.position, rmid1.position)
            );
        // 左腿
        midLhip = Quaternion.Inverse(lhip.rotation) * Quaternion.LookRotation(lhip.position - lknee.position, forward);
        midLknee = Quaternion.Inverse(lknee.rotation) * Quaternion.LookRotation(lknee.position - lfoot.position, forward);
        midLfoot = Quaternion.Inverse(lfoot.rotation) * Quaternion.LookRotation(lfoot.position - ltoe.position, lknee.position-lfoot.position);
        // 右腿
        midRhip = Quaternion.Inverse(rhip.rotation) * Quaternion.LookRotation(rhip.position - rknee.position, forward);
        midRknee = Quaternion.Inverse(rknee.rotation) * Quaternion.LookRotation(rknee.position - rfoot.position, forward);
        midRfoot = Quaternion.Inverse(rfoot.rotation) * Quaternion.LookRotation(rfoot.position - rtoe.position, rknee.position-rfoot.position);
    }

    // 画图
    void DrawPred(List<Vector3> pred3D)
    {
        indicator.position = pred3D[24];
        for (int i = 0; i < 28; i++)
            if (parent[i] != -1)
                Debug.DrawLine(pred3D[i], pred3D[parent[i]], Color.red);
    }

    // 计算三角形法向量
    Vector3 TriangleNormal(Vector3 a, Vector3 b, Vector3 c)
    {
        Vector3 d1 = a - b;
        Vector3 d2 = a - c;

        Vector3 dd = Vector3.Cross(d1, d2);
        dd.Normalize();

        return dd;
    }

    // 更新姿势
    void updatePose(List<Vector3> pred3D)
    {
        /// rShldrBend 0, rForearmBend 1, rHand 2, rThumb2 3, rMid1 4,
        /// lShldrBend 5, lForearmBend 6, lHand 7, lThumb2 8, lMid1 9,
        /// lEar 10, lEye 11, rEar 12, rEye 13, Nose 14,
        /// rThighBend 15, rShin 16, rFoot 17, rToe 18,
        /// lThighBend 19, lShin 20, lFoot 21, lToe 22,    
        /// abdomenUpper 23,
        /// hip 24, head 25, neck 26, spine 27
        /// 
        //////////////////////  更新位置 //////////////////////
        float tallShin = (Vector3.Distance(pred3D[16], pred3D[17]) + Vector3.Distance(pred3D[20], pred3D[21]))/2.0f;
        float tallThigh = (Vector3.Distance(pred3D[15], pred3D[16]) + Vector3.Distance(pred3D[19], pred3D[20]))/2.0f;
        float tallUnity = (Vector3.Distance(lhip.position, lknee.position) + Vector3.Distance(lknee.position, lfoot.position)) / 2.0f +
            (Vector3.Distance(rhip.position, rknee.position) + Vector3.Distance(rknee.position, rfoot.position));
        root.position = pred3D[24] * (tallUnity/(tallThigh+tallShin));

        //////////////////////  更新旋转 //////////////////////
        Vector3 forward = TriangleNormal(pred3D[24], pred3D[19], pred3D[15]);
        // Root
        root.rotation = Quaternion.LookRotation(forward) * Quaternion.Inverse(midRoot);
        // 躯干
        spine.rotation = Quaternion.LookRotation(pred3D[27] - pred3D[26], forward) * Quaternion.Inverse(midSpine);
        neck.rotation = Quaternion.LookRotation(pred3D[26] - pred3D[25], forward) * Quaternion.Inverse(midNeck);
        // 头部
        head.rotation = Quaternion.LookRotation(pred3D[14] - pred3D[25], TriangleNormal(pred3D[14], pred3D[12], pred3D[10])) * Quaternion.Inverse(midHead);
        // 左臂
        lshoulder.rotation = Quaternion.LookRotation(pred3D[5] - pred3D[6], forward) * Quaternion.Inverse(midLshoulder);
        lelbow.rotation = Quaternion.LookRotation(pred3D[6] - pred3D[7], forward) * Quaternion.Inverse(midLelbow);
        lhand.rotation = Quaternion.LookRotation(
            pred3D[8] - pred3D[9],
            TriangleNormal(pred3D[7], pred3D[8], pred3D[9]))*Quaternion.Inverse(midLhand);
        // 右臂
        rshoulder.rotation = Quaternion.LookRotation(pred3D[0] - pred3D[1], forward) * Quaternion.Inverse(midRshoulder);
        relbow.rotation = Quaternion.LookRotation(pred3D[1] - pred3D[2], forward) * Quaternion.Inverse(midRelbow);
        rhand.rotation = Quaternion.LookRotation(
            pred3D[3] - pred3D[4],
            TriangleNormal(pred3D[2], pred3D[3], pred3D[4])) * Quaternion.Inverse(midRhand);
        // 左腿
        lhip.rotation = Quaternion.LookRotation(pred3D[19] - pred3D[20], forward)*Quaternion.Inverse(midLhip);
        lknee.rotation = Quaternion.LookRotation(pred3D[20] - pred3D[21], forward)*Quaternion.Inverse(midLknee);
        lfoot.rotation = Quaternion.LookRotation(pred3D[21] - pred3D[22], pred3D[20]-pred3D[21])*Quaternion.Inverse(midLfoot);
        // 右腿
        rhip.rotation = Quaternion.LookRotation(pred3D[15] - pred3D[16], forward)*Quaternion.Inverse(midRhip);
        rknee.rotation = Quaternion.LookRotation(pred3D[16] - pred3D[17], forward)*Quaternion.Inverse(midRknee);
        rfoot.rotation = Quaternion.LookRotation(pred3D[17] - pred3D[18], pred3D[16]-pred3D[17])*Quaternion.Inverse(midRfoot);
    }

    // Update is called once per frame
    void Update()
    {
        List<Vector3> curpos = pose3D[frame/playRatio];
        updatePose(curpos);
        if (frame / playRatio < pose3D.Count)
            frame = frame + 1;
        else
            frame = 0;
        DrawPred(curpos);
    }
}
