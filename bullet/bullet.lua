local ffi = require("ffi")
ffi.cdef([[void(btPolyhedralConvexAabbCachingShape_recalcLocalAabb)(void*);
void(btBvhTriangleMeshShape_setOptimizedBvh)(void*,void*);
void*(btEmptyAlgorithm_new)(const void*);
void*(btDefaultCollisionConfiguration_new)();
void(btTranslationalLimitMotor2_getSpringStiffness)(void*,void*);
void(btSoftBody_setTimeacc)(void*,float);
float*(btSoftBody_Note_getCoords)(void*);
void*(btSoftBody_getNotes)(void*);
void*(btSliderConstraint_new)(void*,void*,const void*,const void*,bool);
void(btAABB_appy_transform)(void*,const void*);
int(btConvex2dConvex2dAlgorithm_CreateFunc_getMinimumPointsPerturbationThreshold)(void*);
void(btRotationalLimitMotor_setLimitSoftness)(void*,float);
void(btRigidBody_computeGyroscopicImpulseImplicit_World)(void*,float,void*);
void*(btWorldImporter_createGearConstraint)(void*,void*,void*,const void*,const void*,float);
bool(btRotationalLimitMotor2_getSpringStiffnessLimited)(void*);
void(btGImpactCollisionAlgorithm_gimpact_vs_concave)(void*,const void*,const void*,const void*,const void*,bool);
void*(btTriangleIndexVertexArray_getIndexedMeshArray)(void*);
void(btCollisionConfiguration_delete)(void*);
const void*(btCollisionWorld_LocalConvexResult_getHitCollisionObject)(void*);
void(btMultiBodyFixedConstraint_getPivotInB)(void*,void*);
void(btCollisionObject_setWorldArrayIndex)(void*,int);
void(btCollisionWorld_LocalConvexResult_delete)(void*);
void(btPrimitiveManagerBase_get_primitive_triangle)(void*,int,void*);
void*(btContinuousConvexCollision_new)(const void*,const void*,void*,void*);
bool(btCollisionObject_isActive)(void*);
int(btRotationalLimitMotor2_getCurrentLimit)(void*);
float(btRaycastVehicle_getSteeringValue)(void*,int);
void(btJointFeedback_getAppliedTorqueBodyB)(void*,void*);
float(btRotationalLimitMotor2_getServoTarget)(void*);
void*(btGImpactCollisionAlgorithm_internalGetResultOut)(void*);
void(btDbvt_sStkNPS_delete)(void*);
void(btBroadphaseInterface_rayTest2)(void*,const void*,const void*,void*,const void*);
void(btHingeConstraint_getInfo2InternalUsingFrameOffset)(void*,void*,const void*,const void*,const void*,const void*);
void(btCollisionWorld_LocalRayResult_getHitNormalLocal)(void*,void*);
void(btManifoldPoint_setAppliedImpulseLateral1)(void*,float);
int(btSoftBody_getTetraVertexNormalData2)(void*,float*,float*);
void(btCollisionWorld_contactPairTest)(void*,void*,void*,void*);
void(btMLCPSolver_setMLCPSolver)(void*,void*);
void(btOverlappingPairCache_setInternalGhostPairCallback)(void*,void*);
void(btAlignedObjectArray_btSoftBody_JointPtr_resizeNoInitialize)(void**,int);
void*(btUnionFind_new)();
int(btMultiBodySolverConstraint_getJacAindex)(void*);
float(btMultiSphereShape_getSphereRadius)(void*,int);
void(btTranslationalLimitMotor_getAccumulatedImpulse)(void*,void*);
void(btMultiBody_setBaseInertia)(void*,const void*);
bool(btCollisionShape_isInfinite)(void*);
void*(btConeTwistConstraint_new2)(void*,const void*);
void(btRaycastVehicle_setPitchControl)(void*,float);
void*(btTriangleIndexVertexArray_new)();
void(btDbvt_clone)(void*,void*);
void(btStridingMeshInterface_unLockReadOnlyVertexBase)(void*,int);
void(btTetrahedronShapeEx_setVertices)(void*,const void*,const void*,const void*,const void*);
float(btContactSolverInfoData_getSingleAxisRollingFrictionThreshold)(void*);
void*(btCollisionShape_getUserPointer)(void*);
void*(btBvhTriangleMeshShape_new2)(void*,bool,const void*,const void*,bool);
void(btCollisionWorld_RayResultCallback_setCollisionFilterMask)(void*,int);
void*(btSoftBody_Feature_getMaterial)(void*);
void(btGeneric6DofSpring2Constraint_setLimitReversed)(void*,int,float,float);
const void*(btManifoldResult_getBody0Wrap)(void*);
void*(btQuantizedBvh_getLeafNodeArray)(void*);
void*(btCollisionAlgorithmCreateFunc_CreateCollisionAlgorithm)(void*,void*,const void*,const void*);
float(btSliderConstraint_getSoftnessDirLin)(void*);
const void*(btConvexConvexAlgorithm_getManifold)(void*);
float(btRotationalLimitMotor2_getTargetVelocity)(void*);
const void*(btGImpactBvh_get_node_pointer)(void*,int);
void(btRotationalLimitMotor2_setSpringStiffnessLimited)(void*,bool);
void(btSoftBody_Body_applyAImpulse)(void*,const void*);
void(btBroadphaseProxy_setClientObject)(void*,void*);
float(btSoftBody_getTotalMass)(void*);
void(btDispatcherInfo_setTimeOfImpact)(void*,float);
float(btSoftBody_Config_getKSK_SPLT_CL)(void*);
float(btSoftBody_Config_getMaxvolume)(void*);
void*(btMotionStateWrapper_new)(void*,void*);
void(btRaycastVehicle_getForwardVector)(void*,void*);
void*(btAlignedObjectArray_btCollisionObjectPtr_at)(void*,int);
void*(btManifoldPoint_getUserPersistentData)(void*);
void*(btRotationalLimitMotor2_new)();
void(btRotationalLimitMotor2_setSpringDamping)(void*,float);
void(btTranslationalLimitMotor2_getEquilibriumPoint)(void*,void*);
void(btConvexTriangleCallback_getAabbMax)(void*,void*);
void(btHeightfieldTerrainShape_setUseDiamondSubdivision2)(void*,bool);
void(btHingeConstraint_setLimit2)(void*,float,float,float);
float(btManifoldPoint_getCombinedContactDamping1)(void*);
void*(btPersistentManifold_new2)(const void*,const void*,int,float,float);
void(btGeneric6DofConstraint_setFrames)(void*,const void*,const void*);
void(btCollisionObject_setActivationState)(void*,int);
bool(btSoftBody_Joint_getDelete)(void*);
void(btSoftBody_Body_linearVelocity)(void*,void*);
void(btTranslationalLimitMotor_getStopERP)(void*,void*);
void*(btSoftBody_getNodes)(void*);
void*(btCollisionAlgorithmCreateFunc_new)();
void(btBroadphaseInterface_destroyProxy)(void*,void*,void*);
void*(btDefaultCollisionConstructionInfo_new)();
void(btCollisionWorld_RayResultCallback_setFlags)(void*,unsigned int);
void(btCharacterControllerInterface_setUpInterpolate)(void*,bool);
void*(btCollisionWorld_AllHitsRayResultCallback_getCollisionObjects)(void*);
int(btConvexShape_getNumPreferredPenetrationDirections)(void*);
void*(btSoftBody_Impulse_operator_m)(void*,float);
void(btConvexPolyhedron_initialize)(void*);
void(btDbvtAabbMm_Maxs)(void*,void*);
int(btSliderConstraint_getFlags)(void*);
void*(btMultiBodyJointMotor_new)(void*,int,float,float);
void(btAABB_increment_margin)(void*,float);
int(btDbvtNodePtr_array_index_of)(void**,void*,int);
void(btCapsuleShape_deSerializeFloat)(void*,void*);
void(btSoftBody_Node_setN)(void*,const void*);
void(btCollisionWorld_delete)(void*);
void(btDbvt_update3)(void*,void*,int);
void*(btConvexPolyhedron_new)();
void(btBvhTriangleMeshShape_serializeSingleTriangleInfoMap)(void*,void*);
float(btHingeConstraint_getLimitBiasFactor)(void*);
void(btTriangleShapeEx_buildTriPlane)(void*,void*);
float(btHingeConstraint_getLowerLimit)(void*);
void*(btDbvt_getFree)(void*);
void(btCollisionObject_setAnisotropicFriction)(void*,const void*,int);
float(btTriangleInfoMap_getConvexEpsilon)(void*);
void(btPoint2PointConstraint_setPivotA)(void*,const void*);
void*(btDbvtAabbMm_new)();
bool(btRigidBody_wantsSleeping)(void*);
void*(btConvexPointCloudShape_new)();
void(btConvexHullShape_getScaledPoint)(void*,int,void*);
void(btContactSolverInfoData_setFriction)(void*,float);
int(btSequentialImpulseConstraintSolver_btRandInt2)(void*,int);
void(btDbvtNode_setParent)(void*,void*);
void(btGearConstraint_getAxisA)(void*,void*);
int(btVoronoiSimplexSolver_getSimplex)(void*,void*,void*,void*);
void(btDefaultMotionState_setCenterOfMassOffset)(void*,const void*);
void(btConeTwistConstraint_setFrames)(void*,const void*,const void*);
void(btBvhTree_clearNodes)(void*);
void*(btSubSimplexClosestResult_new)();
int(btChunk_getNumber)(void*);
void(btAABB_invalidate)(void*);
int(btGImpactMeshShapePart_TrimeshPrimitiveManager_get_vertex_count)(void*);
float(btTypedConstraint_btConstraintInfo2_getErp)(void*);
void(btGjkPairDetector_setDegenerateSimplex)(void*,int);
float(btDispatcherInfo_getTimeStep)(void*);
void(btBvhTree_getNodeBound)(void*,int,void*);
int(btSoftBody_RayFromToCaster_getTests)(void*);
void(btDefaultCollisionConstructionInfo_setUseEpaPenetrationAlgorithm)(void*,int);
void(btSoftBody_getAabb)(void*,void*,void*);
int(btPersistentManifold_getIndex1a)(void*);
void*(btSoftBody_getWorldInfo)(void*);
void(btRigidBody_setLinearVelocity)(void*,const void*);
void(btRigidBody_getCenterOfMassPosition)(void*,void*);
void(btDbvt_setRoot)(void*,void*);
bool(btMultiBody_isPosUpdated)(void*);
int(btOptimizedBvhNode_getSubPart)(void*);
void*(btCollisionAlgorithmConstructionInfo_new)();
void(btWheelInfo_getWheelAxleCS)(void*,void*);
void(btDynamicsWorld_addConstraint)(void*,void*,bool);
int(btGImpactCollisionAlgorithm_getFace1)(void*);
void(btCollisionWorld_ConvexResultCallback_setCollisionFilterMask)(void*,int);
bool(btMultiBody_getCanSleep)(void*);
float(btSliderConstraint_getSoftnessOrthoAng)(void*);
int(btTypedConstraint_getUid)(void*);
void(btMultiBodyConstraintSolver_solveMultiBodyGroup)(void*,void**,int,void**,int,void**,int,void**,int,const void*,void*,void*);
void(btGeneric6DofConstraint_getInfo1NonVirtual)(void*,void*);
void*(btChunk_new)();
void(btWheelInfoConstructionInfo_setWheelsDampingRelaxation)(void*,float);
unsigned int(btDbvt_getOpath)(void*);
void(btCollisionWorld_ContactResultCallback_setClosestDistanceThreshold)(void*,float);
int(btBU_Simplex1to4_getIndex)(void*,int);
void(btAABB_projection_interval)(void*,const void*,float*,float*);
void(btContactSolverInfoData_setSolverMode)(void*,int);
void(btSubSimplexClosestResult_setBarycentricCoordinates4)(void*,float,float,float);
void(btRigidBody_btRigidBodyConstructionInfo_setAdditionalLinearDampingThresholdSqr)(void*,float);
void(btRaycastVehicle_setBrake)(void*,float,int);
float(btTranslationalLimitMotor_solveLinearAxis)(void*,float,float,void*,const void*,void*,const void*,int,const void*,const void*);
float(btRigidBody_btRigidBodyConstructionInfo_getAdditionalAngularDampingThresholdSqr)(void*);
void(btDefaultMotionState_setGraphicsWorldTrans)(void*,const void*);
float(btConeTwistConstraint_getRelaxationFactor)(void*);
void(btGeneric6DofSpring2Constraint_getLinearLowerLimit)(void*,void*);
void*(btSoftBody_SContact_getFace)(void*);
void(btSoftBody_Cluster_setContainsAnchor)(void*,bool);
bool(btUsageBitfield_getUnused3)(void*);
void(btDbvt_rayTest)(const void*,const void*,const void*,void*);
float(btContactSolverInfoData_getTimeStep)(void*);
void(btQuantizedBvh_reportRayOverlappingNodex)(void*,void*,const void*,const void*);
int(btCollisionObject_getCollisionFlags)(void*);
void(btDiscreteDynamicsWorld_synchronizeSingleMotionState)(void*,void*);
bool(btOverlappingPairCache_hasDeferredRemoval)(void*);
void(btCollisionWorld_RayResultCallback_setCollisionFilterGroup)(void*,int);
void(btBroadphaseProxy_setAabbMax)(void*,const void*);
void(btMultiBody_wakeUp)(void*);
void*(btSoftBody_RContact_getCti)(void*);
void(btDbvt_IClone_CloneLeaf)(void*,void*);
void(btRaycastVehicle_setCoordinateSystem)(void*,int,int,int);
void(btUnionFind_delete)(void*);
void(btCollisionWorld_setBroadphase)(void*,void*);
void*(btWorldImporter_createCapsuleShapeY)(void*,float,float);
const unsigned char*(btMaterialProperties_getTriangleMaterialsBase)(void*);
bool(btAABB_has_collision)(void*,const void*);
void(btCollisionObject_setInterpolationAngularVelocity)(void*,const void*);
void(btGeneric6DofSpring2Constraint_setFrames)(void*,const void*,const void*);
void(btSoftBody_randomizeConstraints)(void*);
void(btBroadphasePair_setPProxy0)(void*,void*);
int(btDbvt_maxdepth)(const void*);
int(btSoftBody_getLinkVertexData)(void*,float*);
void(btGeneric6DofSpring2Constraint_getAxis)(void*,int,void*);
void*(btBU_Simplex1to4_new)();
void(btAlignedObjectArray_btBroadphasePair_push_back)(void*,void*);
void(btOptimizedBvh_build)(void*,void*,bool,const void*,const void*);
void(btHinge2Constraint_getAnchor)(void*,void*);
bool(btCollisionShape_isConvex2d)(void*);
float(btGeneric6DofConstraint_getAngle)(void*,int);
bool(btQuantizedBvhTree_testQuantizedBoxOverlapp)(void*,int,unsigned short*,unsigned short*);
void(btHingeConstraint_setMotorTarget)(void*,float,float);
float(btSoftBody_RContact_getC4)(void*);
int(btAlignedObjectArray_btSoftBody_Tetra_size)(void*);
void(btSoftBody_Impulse_delete)(void*);
int(btGImpactBvh_getEscapeNodeIndex)(void*,int);
float(btSliderConstraint_getDampingLimLin)(void*);
void*(btDefaultCollisionConfiguration_new2)(const void*);
void*(btSoftRigidDynamicsWorld_new)(void*,void*,void*,void*,void*);
bool(btSliderConstraint_getUseLinearReferenceFrameA)(void*);
float(btManifoldPoint_getCombinedRollingFriction)(void*);
void(btHingeConstraint_getFrameOffsetB)(void*,void*);
void(btCollisionShape_serializeSingleShape)(void*,void*);
void(btQuantizedBvhTree_delete)(void*);
void(btDbvtBroadphase_setUpdates_ratio)(void*,float);
bool(btTriangleMesh_getUse4componentVertices)(void*);
void(btCollisionObject_internalSetExtensionPointer)(void*,void*);
void(btDbvtBroadphase_setStageCurrent)(void*,int);
void(btGeneric6DofSpring2Constraint_enableSpring)(void*,int,bool);
void*(btTriangleShapeEx_new2)(const void*,const void*,const void*);
float(btRotationalLimitMotor2_getBounce)(void*);
void(btDbvtBroadphase_setVelocityPrediction)(void*,float);
void*(btGeneric6DofConstraint_new2)(void*,const void*,bool);
float*(btMultiBodyConstraint_jacobianA)(void*,int);
void*(btBoxBoxCollisionAlgorithm_new2)(void*,const void*,const void*,const void*);
float(btSliderConstraint_getSoftnessOrthoLin)(void*);
int(btGeneric6DofConstraint_get_limit_motor_info2)(void*,void*,const void*,const void*,const void*,const void*,const void*,const void*,void*,int,void*,int,int);
void(btNNCGConstraintSolver_setOnlyForNoneContact)(void*,bool);
void*(btDbvt_sStkNP_new)(const void*,unsigned int);
void(btAlignedObjectArray_btSoftBody_Tetra_resizeNoInitialize)(void*,int);
void(btSubSimplexClosestResult_setBarycentricCoordinates3)(void*,float,float);
void(btQuantizedBvh_delete)(void*);
float(btRaycastVehicle_getCurrentSpeedKmHour)(void*);
void(btGeneric6DofConstraint_setAxis)(void*,const void*,const void*);
void(btDiscreteCollisionDetectorInterface_ClosestPointInput_getTransformB)(void*,void*);
const char*(btCollisionShape_serialize)(void*,void*,void*);
void*(btCompoundCollisionAlgorithm_SwappedCreateFunc_new)();
void(btCollisionWorld_ClosestConvexResultCallback_setHitCollisionObject)(void*,const void*);
void(btCollisionWorld_rayTest)(void*,const void*,const void*,void*);
void*(btConvexPolyhedron_getVertices)(void*);
int(btGImpactQuantizedBvh_getEscapeNodeIndex)(void*,int);
int(btDispatcherInfo_getStepCount)(void*);
void*(btCylinderShape_new)(const void*);
int(btSoftBody_Impulse_getAsDrift)(void*);
void(btStridingMeshInterface_calculateAabbBruteForce)(void*,void*,void*);
void*(btWorldImporter_createConeShapeY)(void*,float,float);
float(btTriangleInfoMap_getMaxEdgeAngleThreshold)(void*);
void(btRotationalLimitMotor_setAccumulatedImpulse)(void*,float);
void*(btConvexHullShape_new2)(const float*);
bool(btSubSimplexClosestResult_isValid)(void*);
void*(btRigidBody_new)(const void*);
int(btRaycastVehicle_getUpAxis)(void*);
float(btAngularLimit_getBiasFactor)(void*);
float(btRigidBody_btRigidBodyConstructionInfo_getAdditionalAngularDampingFactor)(void*);
void(btBroadphaseProxy_setAabbMin)(void*,const void*);
void*(btOverlappingPairCache_getOverlappingPairArray)(void*);
void(btOverlappingPairCallback_delete)(void*);
void*(btMultiBody_getBaseCollider)(void*);
int(btSoftBody_CJoint_getMaxlife)(void*);
void*(btMultiBodyJointMotor_new2)(void*,int,int,float,float);
float(btSoftBody_Cluster_getIdmass)(void*);
float(btSoftBody_Config_getKDG)(void*);
const unsigned char*(btGImpactMeshShapePart_TrimeshPrimitiveManager_getIndexbase)(void*);
void(btRigidBody_setMotionState)(void*,void*);
void(btBox2dShape_getPlaneEquation)(void*,void*,int);
void(btRotationalLimitMotor2_setCurrentPosition)(void*,float);
int(btGImpactMeshShapePart_TrimeshPrimitiveManager_getIndicestype)(void*);
float(btUniversalConstraint_getAngle1)(void*);
void(btSoftRigidDynamicsWorld_removeSoftBody)(void*,void*);
void(btGjkPairDetector_setCurIter)(void*,int);
bool(btShapeHull_buildHull)(void*,float);
void(btManifoldPoint_delete)(void*);
bool(btAABB_collide_ray)(void*,const void*,const void*);
void(btGImpactCollisionAlgorithm_setPart1)(void*,int);
void(btSoftBody_appendNote)(void*,const char*,const void*,void*);
void*(btGjkConvexCast_new)(const void*,const void*,void*);
void(btKinematicCharacterController_setMaxJumpHeight)(void*,float);
void(btSoftBodyHelpers_DrawFrame)(void*,void*);
void(btTypedConstraint_delete)(void*);
void(btManifoldPoint_setFrictionCFM)(void*,float);
unsigned int(btDbvtBroadphase_getUpdates_done)(void*);
float(btManifoldPoint_getCombinedFriction)(void*);
void(btCollisionObject_setCompanionId)(void*,int);
void(btConstraintSolver_reset)(void*);
float(btConeTwistConstraint_getSwingSpan1)(void*);
void(btAABB_copy_with_margin)(void*,const void*,float);
int(btMultibodyLink_getCfgOffset)(void*);
void(btSoftBody_Cluster_setCollide)(void*,bool);
void(btRigidBody_setDamping)(void*,float,float);
bool(btCollisionWorld_ConvexResultCallback_needsCollision)(void*,void*);
void(btSoftBody_CJoint_setNormal)(void*,const void*);
void*(btSoftBody_Cluster_getDimpulses)(void*);
void(btRigidBody_btRigidBodyConstructionInfo_delete)(void*);
float*(btTypedConstraint_btConstraintInfo2_getJ2linearAxis)(void*);
void(btRigidBody_btRigidBodyConstructionInfo_setLocalInertia)(void*,const void*);
void(btDbvt_ICollide_Process)(void*,const void*,const void*);
int(btAlignedObjectArray_btSoftBody_MaterialPtr_size)(void*);
void(btUnionFind_Free)(void*);
int(btSoftBody_sRayCast_getFeature)(void*);
void*(btWorldImporter_createSliderConstraint2)(void*,void*,const void*,bool);
void*(btWorldImporter_createMultiSphereShape)(void*,const void*,const float*,int);
float(btRotationalLimitMotor2_getLoLimit)(void*);
void(btMultibodyLink_getCachedWorldTransform)(void*,void*);
void(btGeneric6DofSpringConstraint_setDamping)(void*,int,float);
void(btMultiBody_setupSpherical)(void*,int,float,const void*,int,const void*,const void*,const void*,bool);
void(btSoftBody_Tetra_setRv)(void*,float);
void(btDbvt_update)(void*,void*,void*);
int(btIndexedMesh_getIndexType)(void*);
bool(btMultiBody_getUseGyroTerm)(void*);
void*(btGeneric6DofSpring2Constraint_getRotationalLimitMotor)(void*,int);
void*(btConvexCast_CastResult_new)();
void(btDynamicsWorld_addAction)(void*,void*);
void(btConvexInternalShape_setImplicitShapeDimensions)(void*,const void*);
void*(btSphereSphereCollisionAlgorithm_new2)(const void*);
bool(btGImpactQuantizedBvh_isLeafNode)(void*,int);
void*(btBulletWorldImporter_new)();
void(btDbvtBroadphase_setPaircache)(void*,void*);
void(btRigidBody_getOrientation)(void*,void*);
int(btMultiBodySolverConstraint_getSolverBodyIdA)(void*);
void(btConvexTriangleCallback_clearWrapperData)(void*);
bool(btDbvtNode_isleaf)(void*);
bool(btRotationalLimitMotor2_getServoMotor)(void*);
void(btRotationalLimitMotor2_delete)(void*);
void(btSoftBody_Pose_setScl)(void*,const void*);
void(btSoftBody_appendAngularJoint3)(void*,const void*,void*);
int(btDefaultCollisionConstructionInfo_getDefaultMaxPersistentManifoldPoolSize)(void*);
void(btSoftBodySolver_delete)(void*);
float*(btMultiBody_getJointTorqueMultiDof)(void*,int);
void(btAlignedObjectArray_btVector3_delete)(void*);
int(btAlignedObjectArray_btVector3_size)(void*);
int(btSoftBody_getFaceVertexNormalData2)(void*,float*,float*);
void(btAlignedObjectArray_btVector3_set)(void*,int,const void*);
void*(btWorldImporter_new)(void*);
void(btSoftBody_Joint_Specs_setCfm)(void*,float);
void(btAlignedObjectArray_btVector3_push_back)(void*,const void*);
void*(btHashedOverlappingPairCache_new)();
void*(btAlignedObjectArray_btVector3_new)();
void*(btSoftBody_getLinks)(void*);
void(btSoftBody_Node_setQ)(void*,const void*);
void(btSoftBody_Cluster_setCom)(void*,const void*);
void(btVector3_array_at)(const void*,int,void*);
void*(btBroadphasePair_new)();
void*(btConvexTriangleMeshShape_getMeshInterface)(void*);
void(btSoftBodyNodePtrArray_set)(void*,void*,int);
void*(btSoftBodyNodePtrArray_at)(void*,int);
void*(btMinkowskiPenetrationDepthSolver_new)();
void*(btBU_Simplex1to4_new4)(const void*,const void*,const void*);
bool(btCollisionObject_isKinematicObject)(void*);
void(btGImpactShapeInterface_lockChildShapes)(void*);
int(btBroadphaseProxy_getUniqueId)(void*);
void(btWheelInfo_RaycastInfo_setWheelDirectionWS)(void*,const void*);
void(btStridingMeshInterface_getPremadeAabb)(void*,void*,void*);
void(btWorldImporter_delete)(void*);
float(btSliderConstraint_getAngularPos)(void*);
float(btManifoldPoint_getAppliedImpulse)(void*);
void*(btWorldImporter_createCapsuleShapeX)(void*,float,float);
void*(btContactSolverInfo_new)();
int(btOptimizedBvhNode_getTriangleIndex)(void*);
void(btConeTwistConstraint_setFixThresh)(void*,float);
int(btGjkPairDetector_getCurIter)(void*);
int(btOverlappingPairCache_getNumOverlappingPairs)(void*);
void(btCollisionObject_serializeSingleObject)(void*,void*);
int(btWorldImporter_getVerboseMode)(void*);
int(btDbvtBroadphase_getGid)(void*);
void(btMultiBodySolverConstraint_setJacAindex)(void*,int);
float(btSoftBody_Cluster_getMatching)(void*);
void*(btWorldImporter_getRigidBodyByIndex)(void*,int);
int(btWorldImporter_getNumTriangleInfoMaps)(void*);
int(btMultiBodySolverConstraint_getDeltaVelBindex)(void*);
float(btCollisionWorld_ConvexResultCallback_getClosestHitFraction)(void*);
void(btHeightfieldTerrainShape_setUseZigzagSubdivision2)(void*,bool);
void(btMultiBody_setHasSelfCollision)(void*,bool);
void(btCollisionWorld_LocalConvexResult_setLocalShapeInfo)(void*,void*);
void(btTriangleInfoMap_setEdgeDistanceThreshold)(void*,float);
void(btHinge2Constraint_getAxis2)(void*,void*);
void(btSliderConstraint_setUseFrameOffset)(void*,bool);
int(btWorldImporter_getNumBvhs)(void*);
void(btSoftBodyWorldInfo_delete)(void*);
int(btSoftBody_getFaceVertexData)(void*,float*);
void*(btWorldImporter_getConstraintByName)(void*,const char*);
void(btMaterialProperties_setTriangleMaterialsBase)(void*,const unsigned char*);
void*(btWorldImporter_getCollisionShapeByName)(void*,const char*);
int(btMaterialProperties_getMaterialType)(void*);
float(btSliderConstraint_getRestitutionOrthoLin)(void*);
void(btSparseSdf3_GarbageCollect)(void*,int);
float(btSoftBody_AJoint_IControl_Speed)(void*,void*,float);
void*(btWorldImporter_createTriangleMeshContainer)(void*);
float(btCollisionShape_getAngularMotionDisc)(void*);
void(btAlignedObjectArray_btSoftBody_ClusterPtr_push_back)(void*,void*);
void*(btWorldImporter_createStridingMeshInterfaceData)(void*,void*);
void(btSoftBodyHelpers_Draw)(void*,void*,int);
void*(btWorldImporter_createSphereShape)(void*,float);
float(btSoftBody_Anchor_getInfluence)(void*);
void*(btWorldImporter_createSliderConstraint)(void*,void*,void*,const void*,const void*,bool);
void(btTypedConstraint_btConstraintInfo2_setRowskip)(void*,int);
void(btSoftBody_Impulse_setAsDrift)(void*,int);
void*(btDbvtAabbMm_FromCE)(const void*,const void*);
bool(btTypedConstraint_isEnabled)(void*);
float(btSoftBody_Joint_Specs_getErp)(void*);
void(btDbvtBroadphase_setUpdates_done)(void*,unsigned int);
bool(btCollisionShape_isConvex)(void*);
void(btGhostObject_convexSweepTest)(void*,const void*,const void*,const void*,void*,float);
int(btDispatcher_getNumManifolds)(void*);
void*(btWorldImporter_createPlaneShape)(void*,const void*,float);
int(btDbvt_getLeaves)(void*);
int(btMultibodyLink_getJointType)(void*);
void*(btWorldImporter_createOptimizedBvh)(void*);
void*(btWorldImporter_createMeshInterface)(void*,void*);
void*(btWorldImporter_createHingeConstraint4)(void*,void*,const void*,bool);
int(btDbvtBroadphase_getStageCurrent)(void*);
void*(btWorldImporter_createHingeConstraint3)(void*,void*,const void*);
void(btManifoldResult_setPersistentManifold)(void*,void*);
void*(btWorldImporter_createHingeConstraint2)(void*,void*,void*,const void*,const void*,bool);
void*(btCollisionWorld_LocalShapeInfo_new)();
void(btGeneric6DofSpring2Constraint_setAngularLowerLimitReversed)(void*,const void*);
void(btConvexPlaneCollisionAlgorithm_CreateFunc_setNumPerturbationIterations)(void*,int);
bool(btBulletWorldImporter_loadFile)(void*,const char*);
void*(btWorldImporter_createHingeConstraint)(void*,void*,void*,const void*,const void*);
void(btPolyhedralConvexAabbCachingShape_getNonvirtualAabb)(void*,const void*,void*,void*,float);
void(btManifoldPoint_setUserPersistentData)(void*,void*);
int(btConvexPointCloudShape_getNumPoints)(void*);
void*(btSubSimplexClosestResult_getUsedVertices)(void*);
void*(btWorldImporter_createGeneric6DofSpring2Constraint)(void*,void*,void*,const void*,const void*,int);
void(btMultibodyLink_getAppliedConstraintTorque)(void*,void*);
bool(btGeneric6DofSpringConstraint_isSpringEnabled)(void*,int);
void*(btWorldImporter_createGeneric6DofConstraint)(void*,void*,void*,const void*,const void*,bool);
void*(btWorldImporter_createCylinderShapeY)(void*,float,float);
void(btSoftBody_solveClusters)(const void*);
void*(btWorldImporter_createCylinderShapeX)(void*,float,float);
int(btTriangle_getTriangleIndex)(void*);
void*(btWorldImporter_createConvexTriangleMeshShape)(void*,void*);
int(btTypedConstraint_btConstraintInfo1_getNub)(void*);
void*(btWorldImporter_createConeTwistConstraint2)(void*,void*,const void*);
float(btConeShape_getHeight)(void*);
void(btDbvtAabbMm_Extents)(void*,void*);
void(btDbvtBroadphase_setUpdates_call)(void*,unsigned int);
void(btCollisionObject_setContactStiffnessAndDamping)(void*,float,float);
void*(btWorldImporter_createCompoundShape)(void*);
void*(btSoftBody_getClusters)(void*);
void(btUnionFind_unite)(void*,int,int);
int(btSoftBodySolver_getNumberOfPositionIterations)(void*);
void(btConeTwistConstraint_setMotorTargetInConstraintSpace)(void*,const void*);
void(btManifoldPoint_setCombinedRestitution)(void*,float);
void*(btWorldImporter_createBvhTriangleMeshShape)(void*,void*,void*);
void(btRigidBody_getVelocityInLocalPoint)(void*,const void*,void*);
void*(btTypedConstraint_getUserConstraintPtr)(void*);
void*(btActionInterfaceWrapper_new)(void*,void*);
void*(btWorldImporter_createBoxShape)(void*,const void*);
void(btConvexPolyhedron_setLocalCenter)(void*,const void*);
void(btWheelInfo_delete)(void*);
int(btAlignedObjectArray_btSoftBody_ClusterPtr_size)(void*);
void(btSoftBody_Joint_setSplit)(void*,float);
int(btConeTwistConstraint_getSolveSwingLimit)(void*);
int(btQuantizedBvh_calculateSerializeBufferSizeNew)(void*);
void(btGImpactCompoundShape_addChildShape2)(void*,void*);
void(btWheelInfo_getWorldTransform)(void*,void*);
void(btWheelInfo_setWheelsSuspensionForce)(void*,float);
void*(btCollisionConfiguration_getCollisionAlgorithmPool)(void*);
void(btWheelInfo_setWheelsDampingRelaxation)(void*,float);
void*(btSoftBody_Node_getLeaf)(void*);
void(btDbvt_sStkNPS_setMask)(void*,int);
void(btWheelInfo_setWheelsDampingCompression)(void*,float);
void(btConvexCast_CastResult_drawCoordSystem)(void*,const void*);
void(btWheelInfo_setWheelDirectionCS)(void*,const void*);
void(btMultiBodySolverConstraint_setAppliedPushImpulse)(void*,float);
void(btWheelInfo_setSuspensionStiffness)(void*,float);
void*(btConvexTriangleMeshShape_new)(void*,bool);
void(btWheelInfo_setSuspensionRestLength1)(void*,float);
void(btWheelInfo_setSuspensionRelativeVelocity)(void*,float);
void(btWheelInfo_setSteering)(void*,float);
void(btDbvt_update2)(void*,void*);
float(btAngularLimit_getError)(void*);
void(btDiscreteCollisionDetectorInterface_Result_setShapeIdentifiersB)(void*,int,int);
void(btAngularLimit_fit)(void*,float*);
void*(btConvexHullShape_new3)(const float*,int);
void(btSoftBodyConcaveCollisionAlgorithm_clearCache)(void*);
void(btRaycastVehicle_updateWheelTransformsWS)(void*,void*);
void(btMultiBody_setupPrismatic)(void*,int,float,const void*,int,const void*,const void*,const void*,const void*,bool);
void(btWheelInfo_setMaxSuspensionTravelCm)(void*,float);
void(btTriangleMesh_addIndex)(void*,int);
void(btWheelInfo_setMaxSuspensionForce)(void*,float);
void*(btHingeAccumulatedAngleConstraint_new)(void*,void*,const void*,const void*,const void*,const void*,bool);
void(btActionInterface_delete)(void*);
void(btWheelInfo_setFrictionSlip)(void*,float);
void(btBvhTriangleMeshShape_performRaycast)(void*,void*,const void*,const void*);
void(btRigidBody_btRigidBodyConstructionInfo_setMotionState)(void*,void*);
void(btCollisionWorld_ContactResultCallback_setCollisionFilterGroup)(void*,int);
void(btCollisionShape_getAabb)(void*,const void*,void*,void*);
void(btSoftBody_appendFace2)(void*,int,int,int,void*);
float(btBroadphaseRayCallback_getLambda_max)(void*);
void(btWheelInfo_setDeltaRotation)(void*,float);
void(btWheelInfo_setClippedInvContactDotSuspension)(void*,float);
void(btWheelInfo_setClientInfo)(void*,void*);
void(btWheelInfo_setChassisConnectionPointCS)(void*,const void*);
void*(btTriangleBuffer_new)();
float(btWheelInfo_getSteering)(void*);
void(btSliderConstraint_getCalculatedTransformA)(void*,void*);
int(btCollisionObject_getCompanionId)(void*);
void(btWheelInfo_setBrake)(void*,float);
void(btWheelInfo_setBIsFrontWheel)(void*,bool);
void*(btSimulationIslandManager_getUnionFind)(void*);
void(btWheelInfo_setWorldTransform)(void*,const void*);
void(btMultiBody_addBaseConstraintForce)(void*,const void*);
float(btWheelInfo_getWheelsRadius)(void*);
int(btAlignedObjectArray_btIndexedMesh_size)(void*);
const char*(btCollisionObject_serialize)(void*,void*,void*);
void(btSoftBody_CJoint_setMaxlife)(void*,int);
void*(btAngularLimit_new)();
void(btSoftBody_VSolve_Links)(void*,float);
void(btSoftBody_setSoftBodySolver)(void*,void*);
float(btTriangleInfoMap_getEdgeDistanceThreshold)(void*);
void(btCompoundShapeChild_getTransform)(void*,void*);
void*(btDefaultMotionState_new2)(const void*);
void(btDbvtNode_delete)(void*);
void(btManifoldPoint_getNormalWorldOnB)(void*,void*);
void(btMultiBody_setCompanionId)(void*,int);
void(btSoftBody_appendLinearJoint3)(void*,const void*,void*);
float(btWheelInfo_getSuspensionRelativeVelocity)(void*);
float(btWheelInfo_getSkidInfo)(void*);
float(btMultiBody_getLinearDamping)(void*);
float(btPersistentManifold_getContactBreakingThreshold)(void*);
float(btWheelInfo_getRotation)(void*);
void(btCollisionObject_setDeactivationTime)(void*,float);
float(btWheelInfo_getRollInfluence)(void*);
void(btCollisionObject_setInterpolationWorldTransform)(void*,const void*);
void*(btWheelInfo_getRaycastInfo)(void*);
bool(btBulletWorldImporter_loadFile2)(void*,const char*,const char*);
void(btCollisionShape_calculateTemporalAabb)(void*,const void*,const void*,const void*,float,void*,void*);
void(btRigidBody_setAngularFactor2)(void*,float);
float(btWheelInfo_getMaxSuspensionTravelCm)(void*);
void(btMultiBodySolverConstraint_getRelpos2CrossNormal)(void*,void*);
const void*(btMinkowskiSumShape_getShapeA)(void*);
float(btWheelInfo_getFrictionSlip)(void*);
void*(btTriangleIndexVertexMaterialArray_new2)(int,int*,int,int,float*,int,int,unsigned char*,int,int*,int);
float(btSliderConstraint_getDampingOrthoLin)(void*);
void(btRigidBody_setAngularFactor)(void*,const void*);
void(btCollisionAlgorithmConstructionInfo_setDispatcher1)(void*,void*);
void(btRotationalLimitMotor2_testLimitValue)(void*,float);
void(btDispatcherInfo_setTimeStep)(void*,float);
void(btSliderConstraint_getFrameOffsetA)(void*,void*);
void(btWheelInfo_getChassisConnectionPointCS)(void*,void*);
void(btDbvtBroadphase_setAabbForceUpdate)(void*,void*,const void*,const void*,void*);
void(btConvexPlaneCollisionAlgorithm_CreateFunc_setMinimumPointsPerturbationThreshold)(void*,int);
void*(btConvexTriangleCallback_getManifoldPtr)(void*);
bool(btWheelInfo_getBIsFrontWheel)(void*);
float(btSoftBodyWorldInfo_getWater_offset)(void*);
int(btGImpactCollisionAlgorithm_getPart0)(void*);
bool(btDbvt_update5)(void*,void*,void*,const void*);
void*(btCompoundShapeChild_array_at)(void*,int);
void(btWheelInfo_RaycastInfo_setWheelAxleWS)(void*,const void*);
void(btWheelInfo_RaycastInfo_setSuspensionLength)(void*,float);
void(btWheelInfo_RaycastInfo_setIsInContact)(void*,bool);
void(btWheelInfo_RaycastInfo_setHardPointWS)(void*,const void*);
void*(btBoxShape_new)(const void*);
void*(btGhostObject_getOverlappingObject)(void*,int);
void(btWheelInfo_RaycastInfo_setGroundObject)(void*,void*);
void(btWheelInfo_RaycastInfo_setContactPointWS)(void*,const void*);
void*(btContactSolverInfoData_new)();
void(btWheelInfo_RaycastInfo_setContactNormalWS)(void*,const void*);
void(btWheelInfo_RaycastInfo_getWheelDirectionWS)(void*,void*);
void(btMultiBodySolverConstraint_getAngularComponentA)(void*,void*);
void*(btHeightfieldTerrainShape_new2)(int,int,const void*,float,int,bool,bool);
void(btBroadphaseInterface_calculateOverlappingPairs)(void*,void*);
void(btMultiBodySolverConstraint_setDeltaVelAindex)(void*,int);
void(btMultiBody_applyDeltaVeeMultiDof)(void*,const float*,float);
float(btWheelInfo_RaycastInfo_getSuspensionLength)(void*);
void*(btCompoundCollisionAlgorithm_new)(const void*,const void*,const void*,bool);
const void*(btCollisionObjectWrapper_getParent)(void*);
void*(btGImpactBvh_getGlobalBox)(void*);
bool(btWheelInfo_RaycastInfo_getIsInContact)(void*);
void*(btDbvtBroadphase_getSets)(void*);
void(btWheelInfo_RaycastInfo_getHardPointWS)(void*,void*);
void*(btWheelInfo_RaycastInfo_getGroundObject)(void*);
void(btRigidBody_setMassProps)(void*,float,const void*);
void(btSoftBody_setVolumeDensity)(void*,float);
void(btWheelInfo_RaycastInfo_getContactNormalWS)(void*,void*);
void(btManifoldPoint_setCombinedFriction)(void*,float);
void(btRigidBody_setAngularVelocity)(void*,const void*);
void(btCollisionWorld_addCollisionObject2)(void*,void*,int);
void(btTypedConstraint_setParam2)(void*,int,float,int);
void(btWheelInfoConstructionInfo_setWheelRadius)(void*,float);
const void*(btPolyhedralConvexShape_getConvexPolyhedron)(void*);
void(btWheelInfoConstructionInfo_setWheelDirectionCS)(void*,const void*);
int(btTypedConstraint_getUserConstraintId)(void*);
void(btMotionState_setWorldTransform)(void*,const void*);
void(btWheelInfoConstructionInfo_setWheelAxleCS)(void*,const void*);
void(btWheelInfoConstructionInfo_setSuspensionStiffness)(void*,float);
void(btWheelInfoConstructionInfo_setSuspensionRestLength)(void*,float);
void(btWheelInfoConstructionInfo_setMaxSuspensionTravelCm)(void*,float);
void(btContactSolverInfoData_setDamping)(void*,float);
void(btHingeConstraint_setLimit3)(void*,float,float,float,float);
void(btWheelInfoConstructionInfo_setMaxSuspensionForce)(void*,float);
void*(btMultiBodySliderConstraint_new)(void*,int,void*,const void*,const void*,const void*,const void*,const void*);
void(btWheelInfoConstructionInfo_setFrictionSlip)(void*,float);
float(btSoftBody_Material_getKAST)(void*);
void(btWheelInfoConstructionInfo_setChassisConnectionCS)(void*,const void*);
void(btTriangle_getVertex2)(void*,void*);
void(btWheelInfoConstructionInfo_setBIsFrontWheel)(void*,bool);
void*(btGImpactCompoundShape_getCompoundPrimitiveManager)(void*);
void*(btGImpactCompoundShape_CompoundPrimitiveManager_new3)();
void(btPrimitiveManagerBase_get_primitive_box)(void*,int,void*);
float(btWheelInfoConstructionInfo_getWheelsDampingRelaxation)(void*);
int(btAlignedObjectArray_btPersistentManifoldPtr_size)(void*);
void(btRigidBody_applyTorqueImpulse)(void*,const void*);
float(btWheelInfoConstructionInfo_getWheelRadius)(void*);
int(btPolyhedralConvexShape_getNumPlanes)(void*);
void(btSoftBody_SolverState_setUpdmrg)(void*,float);
void(btWheelInfoConstructionInfo_getWheelAxleCS)(void*,void*);
void(btCollisionObject_getWorldTransform)(void*,void*);
float(btWheelInfoConstructionInfo_getSuspensionStiffness)(void*);
void*(btCylinderShapeX_new)(const void*);
void(btDbvtAabbMm_Lengths)(void*,void*);
void(btHingeConstraint_setMotorTarget2)(void*,const void*,float);
void(btMultiBodyDynamicsWorld_clearMultiBodyForces)(void*);
void(btCollisionObject_getInterpolationLinearVelocity)(void*,void*);
void(btAABB_getMax)(void*,void*);
void(btManifoldPoint_setIndex0)(void*,int);
void(btMLCPSolverInterface_delete)(void*);
float(btWheelInfoConstructionInfo_getSuspensionRestLength)(void*);
void(btMultibodyLink_getEVector)(void*,void*);
int(btGImpactMeshShapePart_TrimeshPrimitiveManager_getType)(void*);
float(btWheelInfoConstructionInfo_getMaxSuspensionForce)(void*);
void(btDbvt_IWriter_WriteLeaf)(void*,const void*,int,int);
float(btWheelInfoConstructionInfo_getFrictionSlip)(void*);
void(btTranslationalLimitMotor2_delete)(void*);
void(btWheelInfoConstructionInfo_getChassisConnectionCS)(void*,void*);
int(btMultibodyLink_getDofCount)(void*);
void(btAABB_delete)(void*);
bool(btWheelInfoConstructionInfo_getBIsFrontWheel)(void*);
void(btDispatcherInfo_setDispatchFunc)(void*,int);
void(btMultiBody_stepPositionsMultiDof)(void*,float,float*,float*);
void*(btSoftBody_AJoint_IControl_new)();
float(btKinematicCharacterController_getMaxPenetrationDepth)(void*);
bool(btVoronoiSimplexSolver_updateClosestVectorAndPoints)(void*);
void(btVoronoiSimplexSolver_setNumVertices)(void*,int);
void*(btSoftBody_ImplicitFnWrapper_new)(void*);
void*(btScaledBvhTriangleMeshShape_new)(void*,const void*);
void(btBvhTree_delete)(void*);
float(btRotationalLimitMotor2_getCurrentLimitErrorHi)(void*);
void(btVoronoiSimplexSolver_setNeedsUpdate)(void*,bool);
void(btVoronoiSimplexSolver_setLastW)(void*,const void*);
void(btSoftBody_appendAngularJoint2)(void*,const void*,void*);
float(btSoftBody_Config_getKSS_SPLT_CL)(void*);
void(btVoronoiSimplexSolver_setCachedV)(void*,const void*);
void(btAABB_setMax)(void*,const void*);
void(btVoronoiSimplexSolver_setCachedP2)(void*,const void*);
void(btVoronoiSimplexSolver_setCachedP1)(void*,const void*);
void*(btGjkPairDetector_new2)(const void*,const void*,int,int,float,float,void*,void*);
void(btVoronoiSimplexSolver_setCachedBC)(void*,const void*);
bool(btSoftBody_cutLink)(void*,const void*,const void*,float);
void(btVoronoiSimplexSolver_reset)(void*);
void(btVoronoiSimplexSolver_removeVertex)(void*,int);
void(btVoronoiSimplexSolver_reduceVertices)(void*,const void*);
float(btContactSolverInfoData_getWarmstartingFactor)(void*);
void(btAngularLimit_test)(void*,float);
void(btSoftBody_defaultCollisionHandler2)(void*,void*);
void(btTranslationalLimitMotor2_getTargetVelocity)(void*,void*);
int(btVoronoiSimplexSolver_pointOutsideOfPlane)(void*,const void*,const void*,const void*,const void*,const void*);
void*(btPoint2PointConstraint_getSetting)(void*);
int(btVoronoiSimplexSolver_numVertices)(void*);
float(btVoronoiSimplexSolver_maxVertex)(void*);
bool(btVoronoiSimplexSolver_inSimplex)(void*,const void*);
void(btChunk_delete)(void*);
void(btMultiBody_setBaseWorldTransform)(void*,const void*);
void(btConvexShape_batchedUnitVectorGetSupportingVertexWithoutMargin)(void*,const void*,void*,int);
float*(btTypedConstraint_btConstraintInfo2_getLowerLimit)(void*);
void(btHinge2Constraint_getAxis1)(void*,void*);
void(btOptimizedBvhNode_delete)(void*);
void(btCollisionWorld_setForceUpdateAllAabbs)(void*,bool);
void(btIndexedMesh_delete)(void*);
void*(btVoronoiSimplexSolver_getSimplexPointsQ)(void*);
void(btBroadphaseInterface_aabbTest)(void*,const void*,const void*,void*);
void(btAlignedObjectArray_btPersistentManifoldPtr_delete)(void*);
void*(btVoronoiSimplexSolver_getSimplexPointsP)(void*);
void*(btCompoundShapeChild_getNode)(void*);
bool(btVoronoiSimplexSolver_getNeedsUpdate)(void*);
int(btSoftBody_rayTest2)(void*,const void*,const void*,float*,int*,int*,bool);
void(btSoftBody_Cluster_setNdamping)(void*,float);
bool(btVoronoiSimplexSolver_getCachedValidClosest)(void*);
void(btVoronoiSimplexSolver_getCachedV)(void*,void*);
void(btUniversalConstraint_getAnchor2)(void*,void*);
void(btMultiBody_clearConstraintForces)(void*);
void(btMultiBody_setJointPos)(void*,int,float);
void(btDefaultMotionState_getCenterOfMassOffset)(void*,void*);
bool(btConeTwistConstraint_isPastSwingLimit)(void*);
void*(btGearConstraint_new)(void*,void*,const void*,const void*,float);
void(btTranslationalLimitMotor_setUpperLimit)(void*,const void*);
void(btDynamicsWorld_setWorldUserInfo)(void*,void*);
int(btTypedConstraint_getUserConstraintType)(void*);
int(btGImpactBvh_getLeftNode)(void*,int);
bool(btDispatcherInfo_getUseEpa)(void*);
void(btCharacterControllerInterface_preStep)(void*,void*);
void(btAlignedObjectArray_btSoftBody_Anchor_resizeNoInitialize)(void*,int);
bool(btVoronoiSimplexSolver_fullSimplex)(void*);
bool(btVoronoiSimplexSolver_emptySimplex)(void*);
void(btManifoldPoint_setLateralFrictionDir2)(void*,const void*);
void(btVoronoiSimplexSolver_compute_points)(void*,void*,void*);
void(btTranslationalLimitMotor2_getSpringDamping)(void*,void*);
void(btCompoundShape_getChildTransform)(void*,int,void*);
void*(btDbvt_insert)(void*,const void*,void*);
bool(btVoronoiSimplexSolver_closestPtPointTetrahedron)(void*,const void*,const void*,const void*,const void*,const void*,void*);
void(btSoftBody_Pose_getScl)(void*,void*);
void(btMultiBodySolverConstraint_getContactNormal2)(void*,void*);
float(btRotationalLimitMotor2_getHiLimit)(void*);
void(btTranslationalLimitMotor2_setServoTarget)(void*,const void*);
void(btSoftBody_Anchor_getLocal)(void*,void*);
void(btVoronoiSimplexSolver_backup_closest)(void*,void*);
void(btContactSolverInfoData_setMinimumSolverBatchSize)(void*,int);
void(btVoronoiSimplexSolver_addVertex)(void*,const void*,const void*,const void*);
void*(btVoronoiSimplexSolver_new)();
void(btSliderConstraint_getCalculatedTransformB)(void*,void*);
void(btSubSimplexClosestResult_setUsedVertices)(void*,const void*);
void(btSubSimplexClosestResult_setDegenerate)(void*,bool);
void(btConstraintSolver_allSolved)(void*,const void*,void*);
void(btSubSimplexClosestResult_setClosestPointOnSimplex)(void*,const void*);
void(btMultiBodyConstraint_internalSetAppliedImpulse)(void*,int,float);
void*(btPrimitiveTriangle_new)();
void(btMultiBody_processDeltaVeeMultiDof2)(void*);
void(btDbvtBroadphase_setPid)(void*,int);
void(btSubSimplexClosestResult_setBarycentricCoordinates5)(void*,float,float,float,float);
const void*(btCollisionWorld_RayResultCallback_getCollisionObject)(void*);
void(btMultiBody_setUserIndex)(void*,int);
void(btSubSimplexClosestResult_setBarycentricCoordinates2)(void*,float);
void(btSubSimplexClosestResult_setBarycentricCoordinates)(void*);
void*(btCollisionWorld_LocalConvexResult_getLocalShapeInfo)(void*);
void*(btWorldImporter_createGeneric6DofSpringConstraint)(void*,void*,void*,const void*,const void*,bool);
void(btGeneric6DofConstraint_getInfo2NonVirtual)(void*,void*,const void*,const void*,const void*,const void*,const void*,const void*);
void(btDbvt_extractLeaves)(const void*,void*);
void(btKinematicCharacterController_setLinearVelocity)(void*,const void*);
void(btSubSimplexClosestResult_getClosestPointOnSimplex)(void*,void*);
float*(btSubSimplexClosestResult_getBarycentricCoords)(void*);
void*(btGeneric6DofSpring2Constraint_new)(void*,void*,const void*,const void*,int);
void(btMultiBodyDynamicsWorld_removeMultiBodyConstraint)(void*,void*);
void(btUsageBitfield_setUsedVertexD)(void*,bool);
void(btConvexCast_CastResult_delete)(void*);
bool(btCollisionObject_isStaticObject)(void*);
void(btUsageBitfield_setUsedVertexC)(void*,bool);
void(btDiscreteCollisionDetectorInterface_delete)(void*);
void(btMultiBody_getBasePos)(void*,void*);
float(btRaycastVehicle_btVehicleTuning_getSuspensionCompression)(void*);
void(btBroadphaseInterface_getBroadphaseAabb)(void*,void*,void*);
void(btMultiBodySolverConstraint_setCfm)(void*,float);
void*(btAlignedObjectArray_btSoftBody_Tetra_at)(void*,int);
void(btUsageBitfield_setUnused4)(void*,bool);
void(btConvexTriangleCallback_setManifoldPtr)(void*,void*);
void*(btTranslationalLimitMotor2_new2)(const void*);
void(btUsageBitfield_setUnused3)(void*,bool);
float(btContactSolverInfoData_getTau)(void*);
float(btHingeConstraint_getHingeAngle)(void*,const void*,const void*);
void(btUsageBitfield_setUnused2)(void*,bool);
void(btUsageBitfield_setUnused1)(void*,bool);
void(btConvexShape_localGetSupportVertexWithoutMarginNonVirtual)(void*,const void*,void*);
bool(btPrimitiveManagerBase_is_trimesh)(void*);
float(btSoftBody_Link_getC2)(void*);
void(btMultiBody_useGlobalVelocities)(void*,bool);
bool(btUsageBitfield_getUsedVertexD)(void*);
bool(btCollisionShape_isConcave)(void*);
bool(btUsageBitfield_getUsedVertexC)(void*);
void*(btSoftBody_RayFromToCaster_getFace)(void*);
bool(btUsageBitfield_getUsedVertexB)(void*);
const void*(btDbvt_sStkNP_getNode)(void*);
float(btMultiBodySolverConstraint_getAppliedImpulse)(void*);
bool(btUsageBitfield_getUnused4)(void*);
void*(btMultiBody_getLink)(void*,int);
void(btPolyhedralConvexShape_getPlane)(void*,void*,void*,int);
bool(btUsageBitfield_getUnused2)(void*);
void*(btBroadphaseRayCallbackWrapper_new)(void*);
bool(btUsageBitfield_getUnused1)(void*);
void(btGImpactBvh_setNodeBound)(void*,int,const void*);
void**(btDispatcher_getInternalManifoldPointer)(void*);
int(btMultibodyLink_getParent)(void*);
void*(btDefaultMotionState_new)();
void(btVehicleRaycaster_delete)(void*);
void*(btVehicleRaycaster_castRay)(void*,const void*,const void*,void*);
void(btSoftBody_RayFromToCaster_setRayTo)(void*,const void*);
void(btVehicleRaycaster_btVehicleRaycasterResult_setHitPointInWorld)(void*,const void*);
void*(btHingeAccumulatedAngleConstraint_new2)(void*,const void*,const void*,bool);
void(btMultiBody_setupFixed)(void*,int,float,const void*,int,const void*,const void*,const void*,bool);
float(btGeneric6DofConstraint_getRelativePivotPosition)(void*,int);
void(btVehicleRaycaster_btVehicleRaycasterResult_setHitNormalInWorld)(void*,const void*);
void(btVehicleRaycaster_btVehicleRaycasterResult_setDistFraction)(void*,float);
void(btVehicleRaycaster_btVehicleRaycasterResult_getHitPointInWorld)(void*,void*);
int(btMultiBodyConstraint_getIslandIdB)(void*);
float(btWheelInfoConstructionInfo_getMaxSuspensionTravelCm)(void*);
bool(btDiscreteDynamicsWorld_getLatencyMotionStateInterpolation)(void*);
float(btVehicleRaycaster_btVehicleRaycasterResult_getDistFraction)(void*);
int(btWorldImporter_getNumRigidBodies)(void*);
void(btUniversalConstraint_setUpperLimit)(void*,float,float);
float(btSoftBodyWorldInfo_getWater_density)(void*);
void(btUniversalConstraint_setLowerLimit)(void*,float,float);
void(btUniversalConstraint_getAxis2)(void*,void*);
void*(btSoftBodyConcaveCollisionAlgorithm_SwappedCreateFunc_new)();
int(btAABB_plane_classify)(void*,const void*);
void(btCollisionWorld_setDebugDrawer)(void*,void*);
void(btMinkowskiSumShape_GetTransformB)(void*,void*);
void(btSoftBody_clusterCom)(void*,int,void*);
void(btMultiBodyConstraint_setPosition)(void*,int,float);
void(btSoftBody_updateClusters)(void*);
void(btCollisionObject_setCollisionShape)(void*,void*);
void*(btUniversalConstraint_new)(void*,void*,const void*,const void*,const void*);
void*(btBulletWorldImporter_new2)(void*);
void(btUnionFind_sortIslands)(void*);
void(btUnionFind_reset)(void*,int);
void(btAlignedObjectArray_btSoftBody_ClusterPtr_resizeNoInitialize)(void*,int);
int(btQuantizedBvhNode_getEscapeIndex)(void*);
bool(btBulletWorldImporter_loadFileFromMemory)(void*,char*,int);
const void*(btBox2dShape_getNormals)(void*);
int(btCollisionObject_calculateSerializeBufferSize)(void*);
int(btDbvt_nearest)(const int*,const void*,float,int,int);
int(btUnionFind_getNumElements)(void*);
bool(btCollisionShape_isCompound)(void*);
void(btRigidBody_getInvInertiaTensorWorld)(void*,void*);
void(btSoftBody_addVelocity)(void*,const void*);
bool(btMultiBody_hasFixedBase)(void*);
float(btSoftBody_RayFromToCaster_rayFromToTriangle)(const void*,const void*,const void*,const void*,const void*,const void*);
void(btMaterialProperties_setMaterialType)(void*,int);
void(btStridingMeshInterface_preallocateIndices)(void*,int);
void(btTriangleMeshShape_getLocalAabbMax)(void*,void*);
void(btMultiBody_setMaxCoordinateVelocity)(void*,float);
void(btMultiBody_stepVelocitiesMultiDof)(void*,float,void*,void*,void*,bool);
void*(btSphereShape_new)(float);
void(btAlignedObjectArray_btSoftBody_Face_resizeNoInitialize)(void*,int);
int(btUnionFind_find)(void*,int,int);
void*(btSphereTriangleCollisionAlgorithm_new)(void*,const void*,const void*,const void*,bool);
void(btManifoldPoint_setLateralFrictionDir1)(void*,const void*);
void(btElement_delete)(void*);
void(btElement_setSz)(void*,int);
void(btElement_setId)(void*,int);
bool(btCollisionObject_isStaticOrKinematicObject)(void*);
int(btElement_getSz)(void*);
int(btElement_getId)(void*);
void(btDbvtBroadphase_benchmark)(void*);
void*(btElement_new)();
int(btStridingMeshInterface_calculateSerializeBufferSize)(void*);
bool(btGeneric6DofSpring2Constraint_isLimited)(void*,int);
float(btUniformScalingShape_getUniformScalingFactor)(void*);
void*(btUniformScalingShape_getChildShape)(void*);
void*(btUniformScalingShape_new)(void*,float);
void(btMultiBody_getLinkTorque)(void*,int,void*);
float(btSoftBody_Joint_Specs_getCfm)(void*);
void(btManifoldPoint_getLateralFrictionDir2)(void*,void*);
unsigned long(btSequentialImpulseConstraintSolver_btRand2)(void*);
unsigned short*(btQuantizedBvhNode_getQuantizedAabbMin)(void*);
void(btConeTwistConstraint_setLimit2)(void*,float,float,float,float,float,float);
unsigned char*(btDefaultSerializer_internalAlloc)(void*,unsigned long);
void(btConvexCast_CastResult_getNormal)(void*,void*);
void(btSoftBody_Joint_delete)(void*);
void(btSoftBody_AJoint_Specs_getAxis)(void*,void*);
void(btDbvtBroadphase_setGid)(void*,int);
void(btSoftBody_Body_angularVelocity2)(void*,void*);
void(btConvexPolyhedron_setMC)(void*,const void*);
float(btAngularLimit_getCorrection)(void*);
void(btWheelInfo_setRotation)(void*,float);
float(btSliderConstraint_getSoftnessDirAng)(void*);
float(btCollisionObject_getContactStiffness)(void*);
void*(btRigidBody_btRigidBodyConstructionInfo_new)(float,void*,void*);
void(btTypedConstraint_setUserConstraintType)(void*,int);
int(btMultiBody_getCompanionId)(void*);
void(btTypedConstraint_setUserConstraintPtr)(void*,void*);
void(btDbvt_IWriter_Prepare)(void*,const void*,int);
void(btTypedConstraint_setupSolverConstraint)(void*,void*,int,int,float);
const char*(btMultiBody_getBaseName)(void*);
void(btWheelInfoConstructionInfo_setWheelsDampingCompression)(void*,float);
void(btTypedConstraint_setParam)(void*,int,float);
void(btTypedConstraint_setOverrideNumSolverIterations)(void*,int);
float(btSliderConstraint_getMaxAngMotorForce)(void*);
void(btTranslationalLimitMotor_getStopCFM)(void*,void*);
int(btGjkPairDetector_getCatchDegeneracies)(void*);
void(btTypedConstraint_setJointFeedback)(void*,void*);
bool(btGImpactQuantizedBvh_boxQueryTrans)(void*,const void*,const void*,void*);
void(btTypedConstraint_setEnabled)(void*,bool);
void(btManifoldPoint_setIndex1)(void*,int);
void(btTypedConstraint_setDbgDrawSize)(void*,float);
void(btDbvt_IWriter_WriteNode)(void*,const void*,int,int,int,int);
void(btTypedConstraint_setBreakingImpulseThreshold)(void*,float);
const char*(btTypedConstraint_serialize)(void*,void*,void*);
void*(btGjkPairDetector_new)(const void*,const void*,void*,void*);
void(btSoftBody_RContact_setC4)(void*,float);
void(btTypedConstraint_internalSetAppliedImpulse)(void*,float);
void(btGeneric6DofConstraint_getAngularUpperLimit)(void*,void*);
void(btGjkPairDetector_getClosestPointsNonVirtual)(void*,const void*,void*,void*);
float(btCylinderShape_getRadius)(void*);
void*(btVoronoiSimplexSolver_getCachedBC)(void*);
void*(btCollisionWorld_LocalConvexResult_new)(const void*,void*,const void*,const void*,float);
bool(btCollisionWorld_ConvexResultCallbackWrapper_needsCollision)(void*,void*);
void*(btTypedConstraint_getRigidBodyB)(void*);
void(btStridingMeshInterface_getLockedReadOnlyVertexIndexBase)(void*,const unsigned char**,int*,int*,int*,const unsigned char**,int*,int*,int*,int);
void(btJointFeedback_setAppliedForceBodyA)(void*,const void*);
float(btTypedConstraint_getParam2)(void*,int,int);
void(btMultiBody_getLinkInertia)(void*,int,void*);
bool(btDispatcherInfo_getEnableSPU)(void*);
float(btTypedConstraint_getParam)(void*,int);
void(btMultiBodySolverConstraint_setLinkA)(void*,int);
void(btSoftBody_setTotalDensity)(void*,float);
void*(btSoftBody_Pose_getPos)(void*);
void*(btTypedConstraint_getJointFeedback)(void*);
bool*(btTranslationalLimitMotor_getEnableMotor)(void*);
void(btMultiBody_addJointTorqueMultiDof2)(void*,int,int,float);
void(btAABB_merge)(void*,const void*);
void*(btSoftBody_Tetra_getLeaf)(void*);
float(btSoftBody_Tetra_getRv)(void*);
float(btSoftBody_Material_getKLST)(void*);
void*(btConeShapeX_new)(float,float);
void(btRigidBody_getGravity)(void*,void*);
void(btTypedConstraint_getInfo2)(void*,void*);
void(btTypedConstraint_getInfo1)(void*,void*);
float(btRotationalLimitMotor_getMaxLimitForce)(void*);
void(btConstraintSetting_setImpulseClamp)(void*,float);
int(btTypedConstraint_getConstraintType)(void*);
void*(btSoftBody_sRayCast_getBody)(void*);
float(btTypedConstraint_getBreakingImpulseThreshold)(void*);
void(btMultiBody_setBaseVel)(void*,const void*);
float(btTypedConstraint_getAppliedImpulse)(void*);
void(btDbvt_sStkNN_delete)(void*);
int(btTypedConstraint_calculateSerializeBufferSize)(void*);
void(btConeShape_setRadius)(void*,float);
void(btTypedConstraint_buildJacobian)(void*);
void*(btCollisionObject_getBroadphaseHandle)(void*);
void(btTypedConstraint_btConstraintInfo2_delete)(void*);
void(btTypedConstraint_btConstraintInfo2_setUpperLimit)(void*,float*);
void*(btAlignedObjectArray_btIndexedMesh_at)(void*,int);
void*(btWorldImporter_createScaledTrangleMeshShape)(void*,void*,const void*);
void(btTypedConstraint_btConstraintInfo2_setNumIterations)(void*,int);
void(btTypedConstraint_btConstraintInfo2_setLowerLimit)(void*,float*);
bool(btCollisionShape_isSoftBody)(void*);
void(btConvexPointCloudShape_getScaledPoint)(void*,int,void*);
float(btMultiBodyConstraint_getMaxAppliedImpulse)(void*);
void(btTypedConstraint_btConstraintInfo2_setJ2linearAxis)(void*,float*);
void(btTypedConstraint_btConstraintInfo2_setJ2angularAxis)(void*,float*);
int(btDbvtBroadphase_getNewpairs)(void*);
void(btMultiBody_setBasePos)(void*,const void*);
const float*(btMultiBody_getVelocityVector)(void*);
void(btTypedConstraint_btConstraintInfo2_setJ1linearAxis)(void*,float*);
void(btTypedConstraint_btConstraintInfo2_setJ1angularAxis)(void*,float*);
void(btTypedConstraint_btConstraintInfo2_setFps)(void*,float);
void(btGImpactBvh_delete)(void*);
void(btTypedConstraint_btConstraintInfo2_setErp)(void*,float);
void(btDispatcher_freeCollisionAlgorithm)(void*,void*);
void(btTypedConstraint_btConstraintInfo2_setDamping)(void*,float);
bool(btQuantizedBvh_isQuantized)(void*);
unsigned int(btQuantizedBvh_calculateSerializeBufferSize)(void*);
void(btTypedConstraint_btConstraintInfo2_setConstraintError)(void*,float*);
void(btTypedConstraint_btConstraintInfo2_setCfm)(void*,float*);
float*(btTypedConstraint_btConstraintInfo2_getUpperLimit)(void*);
int(btSoftBody_clusterCount)(void*);
void*(btDiscreteDynamicsWorld_getCollisionWorld)(void*);
int(btStridingMeshInterface_getNumSubParts)(void*);
int(btMultibodyLink_getFlags)(void*);
float(btManifoldPoint_getCombinedRestitution)(void*);
void(btCollisionObjectWrapper_setPartId)(void*,int);
void(btCharacterControllerInterface_setVelocityForTimeInterval)(void*,const void*,float);
void(btConvexCast_CastResult_DebugDraw)(void*,float);
float*(btTypedConstraint_btConstraintInfo2_getJ2angularAxis)(void*);
void*(btWheelInfoConstructionInfo_new)();
float*(btTypedConstraint_btConstraintInfo2_getJ1angularAxis)(void*);
float(btTypedConstraint_btConstraintInfo2_getFps)(void*);
void*(btGImpactBvh_new2)(void*);
void(btSoftBody_Config_setKKHR)(void*,float);
void*(btSoftBody_Element_getTag)(void*);
void(btCompoundShapeChild_setNode)(void*,void*);
float(btManifoldResult_calculateCombinedContactStiffness)(const void*,const void*);
bool(btDbvtBroadphase_getDeferedcollide)(void*);
bool*(btTranslationalLimitMotor2_getSpringDampingLimited)(void*);
void(btQuantizedBvh_deSerializeFloat)(void*,void*);
void(btContactConstraint_setContactManifold)(void*,void*);
void*(btTypedConstraint_btConstraintInfo2_new)();
void*(btCollisionAlgorithmConstructionInfo_getManifold)(void*);
void(btTypedConstraint_btConstraintInfo1_setNumConstraintRows)(void*,int);
void(btTypedConstraint_btConstraintInfo1_setNub)(void*,int);
void(btMultiBody_worldPosToLocal)(void*,int,const void*,void*);
void*(btTetrahedronShapeEx_new)();
int(btTypedConstraint_btConstraintInfo1_getNumConstraintRows)(void*);
void*(btWorldImporter_createConvexHullShape)(void*);
void*(btTypedConstraint_btConstraintInfo1_new)();
void*(btCapsuleShapeX_new)(float,float);
void(btDefaultMotionState_setUserPointer)(void*,void*);
void*(btConvexTriangleCallback_new)(void*,const void*,const void*,bool);
void(btMultiBodySolverConstraint_setOrgDofIndex)(void*,int);
void(btRigidBody_getAngularFactor)(void*,void*);
void(btJointFeedback_setAppliedTorqueBodyA)(void*,const void*);
void(btKinematicCharacterController_setUp)(void*,const void*);
void(btDbvt_sStkNN_setB)(void*,const void*);
bool(btRigidBody_btRigidBodyConstructionInfo_getAdditionalDamping)(void*);
void(btPoint2PointConstraint_getInfo1NonVirtual)(void*,void*);
void*(btCollisionWorld_LocalRayResult_new)(const void*,void*,const void*,float);
void(btJointFeedback_getAppliedForceBodyA)(void*,void*);
void*(btJointFeedback_new)();
bool(btTriangleShapeEx_overlap_test_conservative)(void*,const void*);
void(btTriangleShapeEx_applyTransform)(void*,const void*);
void*(btTriangleShapeEx_new3)(const void*);
void(btManifoldPoint_setContactMotion1)(void*,float);
void(btSoftBody_sCti_getNormal)(void*,void*);
void(btConvexTriangleCallback_clearCache)(void*);
void(btContactSolverInfoData_setGlobalCfm)(void*,float);
void(btCollisionObject_setCcdSweptSphereRadius)(void*,float);
void*(btTriangleShapeEx_new)();
void(btPrimitiveTriangle_delete)(void*);
void*(btCollisionWorld_AllHitsRayResultCallback_getHitFractions)(void*);
void(btPrimitiveTriangle_setPlane)(void*,const void*);
void(btPrimitiveTriangle_setMargin)(void*,float);
void(btRaycastVehicle_setSteeringValue)(void*,float,int);
void(btPrimitiveTriangle_setDummy)(void*,float);
void(btPersistentManifold_replaceContactPoint)(void*,const void*,int);
void(btTranslationalLimitMotor2_setCurrentLimitErrorHi)(void*,const void*);
bool(btPrimitiveTriangle_overlap_test_conservative)(void*,const void*);
void*(btPrimitiveTriangle_getVertices)(void*);
void(btPrimitiveTriangle_getPlane)(void*,void*);
void*(btCollisionWorld_ClosestRayResultCallback_new)(const void*,const void*);
void(btMinkowskiSumShape_setTransformB)(void*,const void*);
void*(btSoftBody_Body_getSoft)(void*);
void(btGImpactBvh_buildSet)(void*);
void*(btDbvt_sStkNPS_new)();
float(btPrimitiveTriangle_getMargin)(void*);
float(btSoftBody_getVolume)(void*);
void(btMultiBody_setUseGyroTerm)(void*,bool);
void*(btDispatcher_getManifoldByIndexInternal)(void*,int);
int(btIndexedMesh_getNumVertices)(void*);
void(btRaycastVehicle_setUserConstraintType)(void*,int);
int(btPrimitiveTriangle_clip_triangle)(void*,void*,void*);
void*(btSimulationIslandManager_new)();
void(btCollisionDispatcher_registerCollisionCreateFunc)(void*,int,int,void*);
float(btSoftBody_Config_getKPR)(void*);
void(btPointCollector_getPointInWorld)(void*,void*);
void*(btDbvtNodePtr_array_at)(void**,int);
void(btHingeConstraint_setAxis)(void*,void*);
void(btCollisionObjectWrapper_setShape)(void*,const void*);
void*(btCollisionConfiguration_getCollisionAlgorithmCreateFunc)(void*,int,int);
void(btPoint2PointConstraint_getInfo2NonVirtual)(void*,void*,const void*,const void*);
void*(btBU_Simplex1to4_new5)(const void*,const void*,const void*,const void*);
void(btCollisionObject_setContactProcessingThreshold)(void*,float);
bool(btSoftBody_checkLink)(void*,const void*,const void*);
void(btOptimizedBvhNode_setTriangleIndex)(void*,int);
float(btRigidBody_btRigidBodyConstructionInfo_getAdditionalDampingFactor)(void*);
void(btCollisionWorld_LocalConvexResult_setHitNormalLocal)(void*,const void*);
void(btGeneric6DofConstraint_calculateTransforms2)(void*);
int(btRigidBody_getFlags)(void*);
void*(btTriangleShape_getVertices1)(void*);
const float*(btTriangleShape_getVertexPtr)(void*,int);
void(btTriangleShape_getPlaneEquation)(void*,int,void*,void*);
void(btTriangleShape_calcNormal)(void*,void*);
float(btRigidBody_getLinearDamping)(void*);
void*(btTriangleShape_new2)(const void*,const void*,const void*);
void*(btMultiBodyConstraintSolver_new)();
void(btKinematicCharacterController_setMaxPenetrationDepth)(void*,float);
float(btSoftBody_Link_getRl)(void*);
void(btGeneric6DofConstraint_updateRHS)(void*,float);
void*(btTriangleShape_new)();
int(btWorldImporter_getNumConstraints)(void*);
void(btTriangleMeshShape_localGetSupportingVertexWithoutMargin)(void*,const void*,void*);
void(btKinematicCharacterController_getLinearVelocity)(void*,void*);
const void*(btBvhTree_get_node_pointer)(void*);
void(btSoftBodyWorldInfo_setAir_density)(void*,float);
int(btAlignedObjectArray_btSoftBody_Note_size)(void*);
void(btConeTwistConstraint_GetPointForAngle)(void*,float,float,void*);
void(btManifoldPoint_getPositionWorldOnB)(void*,void*);
void(btStridingMeshInterface_unLockVertexBase)(void*,int);
void*(btGImpactMeshShape_getMeshInterface)(void*);
int(btMaterialProperties_getTriangleMaterialStride)(void*);
float(btSoftBody_RayFromToCaster_getMint)(void*);
void(btSoftBody_Face_setNormal)(void*,const void*);
void*(btSoftBody_getTag)(void*);
void(btGImpactBvh_setPrimitiveManager)(void*,void*);
void*(btSoftBodyConcaveCollisionAlgorithm_CreateFunc_new)();
int(btTriangleMesh_findOrAddVertex)(void*,const void*,bool);
void(btTriangleMesh_addTriangleIndices)(void*,int,int,int);
bool(btCollisionObject_checkCollideWith)(void*,const void*);
void(btAlignedObjectArray_btSoftBodyPtr_resizeNoInitialize)(void*,int);
void(btDiscreteCollisionDetectorInterface_Result_delete)(void*);
float(btCollisionObject_getContactProcessingThreshold)(void*);
void*(btTriangleMesh_new)(bool,bool);
float(btSoftBody_Tetra_getC1)(void*);
void(btSoftBody_Cluster_setFramexform)(void*,const void*);
void(btTriangleInfoMap_setPlanarEpsilon)(void*,float);
void(btRigidBody_translate)(void*,const void*);
void(btGeneric6DofSpring2Constraint_calculateTransforms)(void*,const void*,const void*);
void(btTriangleInfoMap_setMaxEdgeAngleThreshold)(void*,float);
bool(btDbvt_ICollide_Descent)(void*,const void*);
int(btWorldImporter_getNumCollisionShapes)(void*);
void(btTriangleInfoMap_setConvexEpsilon)(void*,float);
const char*(btTriangleInfoMap_serialize)(void*,void*,void*);
void(btNodeOverlapCallback_delete)(void*);
float(btTriangleInfoMap_getPlanarEpsilon)(void*);
int(btMultiBody_getUserIndex2)(void*);
void*(btManifoldPoint_new2)(const void*,const void*,const void*,float);
float(btTriangleInfoMap_getEqualVertexThreshold)(void*);
void(btRigidBody_removeConstraintRef)(void*,void*);
void(btTriangleInfo_setEdgeV1V2Angle)(void*,float);
void(btContactSolverInfoData_delete)(void*);
void(btSoftBody_RContact_getC0)(void*,void*);
void(btMultiBody_setNumLinks)(void*,int);
void*(btGImpactCompoundShape_new)(bool);
int(btContactSolverInfoData_getMinimumSolverBatchSize)(void*);
float(btSliderConstraint_getSoftnessLimAng)(void*);
void(btTriangleInfo_delete)(void*);
void*(btBU_Simplex1to4_new3)(const void*,const void*);
void(btMultiBody_getRVector)(void*,int,void*);
void(btTriangleInfo_setFlags)(void*,int);
void(btTriangleInfo_setEdgeV2V0Angle)(void*,float);
int(btTriangleInfoMap_calculateSerializeBufferSize)(void*);
bool(btAngularLimit_isLimit)(void*);
int(btTriangleInfo_getFlags)(void*);
void(btStridingMeshInterface_InternalProcessAllTriangles)(void*,void*,const void*,const void*);
float(btTriangleInfo_getEdgeV2V0Angle)(void*);
float(btTriangleInfo_getEdgeV1V2Angle)(void*);
void*(btSoftBody_Body_getRigid)(void*);
float(btTriangleInfo_getEdgeV0V1Angle)(void*);
void(btOverlappingPairCache_cleanProxyFromPairs)(void*,void*,void*);
void*(btTriangleInfo_new)();
void(btBvhTree_setNodeBound)(void*,int,const void*);
void(btConvexCast_CastResult_setFraction)(void*,float);
void(btConvexCast_CastResult_getHitPoint)(void*,void*);
int(btAlignedObjectArray_btSoftBody_JointPtr_size)(void**);
void(btMultibodyLink_getAxisTop)(void*,int,void*);
void(btSliderConstraint_setMaxAngMotorForce)(void*,float);
void(btTriangleIndexVertexMaterialArray_addMaterialProperties)(void*,const void*,int);
bool(btPoint2PointConstraint_getUseSolveConstraintObsolete)(void*);
float(btRaycastVehicle_btVehicleTuning_getFrictionSlip)(void*);
void*(btTriangleIndexVertexMaterialArray_new)();
void(btMaterialProperties_delete)(void*);
void(btGeneric6DofConstraint_getCalculatedTransformB)(void*,void*);
void*(btIDebugDrawWrapper_new)(void*,void*,void*,void*,void*,void*,void*,void*,void*,void*,void*,void*,void*,void*,void*,void*);
void(btMaterialProperties_setTriangleType)(void*,int);
void*(btConstraintSetting_new)();
void*(btWorldImporter_getConstraintByIndex)(void*,int);
void(btMaterialProperties_setNumTriangles)(void*,int);
void(btOverlapFilterCallback_delete)(void*);
void(btMaterialProperties_setNumMaterials)(void*,int);
bool(btBroadphaseProxy_isNonMoving)(int);
void*(btUnionFind_getElement)(void*,int);
void(btMaterialProperties_setMaterialStride)(void*,int);
void(btDbvt_sStkNN_setA)(void*,const void*);
void(btConeTwistConstraint_setMaxMotorImpulse)(void*,float);
void(btSoftBody_applyClusters)(void*,bool);
void(btCollisionObject_setUserPointer)(void*,void*);
void(btMaterialProperties_setMaterialBase)(void*,const unsigned char*);
int(btMaterialProperties_getTriangleType)(void*);
void(btRotationalLimitMotor2_setMaxMotorForce)(void*,float);
bool(btOptimizedBvh_serializeInPlace)(void*,void*,unsigned int,bool);
void*(btSoftBody_Config_getDsequence)(void*);
int(btMaterialProperties_getMaterialStride)(void*);
void(btAlignedObjectArray_btSoftBody_Link_set)(void*,void*,int);
void(btMultiBodyDynamicsWorld_removeMultiBody)(void*,void*);
void*(btMaterialProperties_new)();
void(btTriangleIndexVertexArray_addIndexedMesh)(void*,const void*,int);
void(btCollisionObject_setUserIndex)(void*,int);
void(btMultiBody_getBaseInertia)(void*,void*);
void(btIndexedMesh_setVertexStride)(void*,int);
float*(btMultiBody_getJointPosMultiDof)(void*,int);
void(btDispatcher_releaseManifold)(void*,void*);
bool(btGImpactShapeInterface_needsRetrieveTetrahedrons)(void*);
void(btIndexedMesh_setTriangleIndexStride)(void*,int);
void*(btTranslationalLimitMotor_new)();
void(btMultiBody_getLinkForce)(void*,int,void*);
void(btIndexedMesh_setTriangleIndexBase)(void*,const unsigned char*);
void(btIndexedMesh_setNumVertices)(void*,int);
void(btIndexedMesh_setNumTriangles)(void*,int);
void(btGjkPairDetector_getCachedSeparatingAxis)(void*,void*);
void(btTranslationalLimitMotor2_setCurrentLimitError)(void*,const void*);
void(btRigidBody_predictIntegratedTransform)(void*,float,void*);
void(btCollisionObject_setFriction)(void*,float);
void(btGImpactShapeInterface_getChildAabb)(void*,int,const void*,void*,void*);
void(btConeTwistConstraint_setMotorTarget)(void*,const void*);
bool*(btTranslationalLimitMotor2_getEnableSpring)(void*);
bool(btHingeConstraint_hasLimit)(void*);
void(btGeneric6DofConstraint_calcAnchorPos)(void*);
int(btGImpactBvh_getNodeCount)(void*);
float(btConstraintSetting_getImpulseClamp)(void*);
void(btMultiBodySolverConstraint_setLowerLimit)(void*,float);
const unsigned char*(btIndexedMesh_getVertexBase)(void*);
int(btIndexedMesh_getTriangleIndexStride)(void*);
bool(btGImpactShapeInterface_hasBoxSet)(void*);
void*(btDispatcher_getNewManifold)(void*,const void*,const void*);
float(btContactSolverInfoData_getSor)(void*);
const unsigned char*(btIndexedMesh_getTriangleIndexBase)(void*);
void(btPrimitiveTriangle_get_edge_plane)(void*,int,void*);
int(btIndexedMesh_getNumTriangles)(void*);
void(btCollisionWorld_debugDrawObject)(void*,const void*,const void*,const void*);
void*(btIndexedMesh_new)();
void(btDbvtBroadphase_setReleasepaircache)(void*,bool);
bool(btGeneric6DofSpring2Constraint_matrixToEulerXZY)(const void*,void*);
void*(btInternalTriangleIndexCallbackWrapper_new)(void*);
void(btTriangleCallback_delete)(void*);
void(btRigidBody_applyCentralForce)(void*,const void*);
void(btCollisionWorld_AllHitsRayResultCallback_getRayToWorld)(void*,void*);
const void*(btTriangleBuffer_getTriangle)(void*,int);
bool(btHashedOverlappingPairCache_needsBroadphaseCollision)(void*,void*,void*);
void(btGhostObject_rayTest)(void*,const void*,const void*,void*);
void(btHeightfieldTerrainShape_setUseDiamondSubdivision)(void*);
int(btTriangleBuffer_getNumTriangles)(void*);
bool(btQuantizedBvhTree_isLeafNode)(void*,int);
void(btTranslationalLimitMotor_setDamping)(void*,float);
void(btTriangle_delete)(void*);
void(btTriangle_setVertex2)(void*,const void*);
void(btPersistentManifold_delete)(void*);
void(btTriangle_setVertex1)(void*,const void*);
void(btTriangle_setVertex0)(void*,const void*);
void(btTriangle_setTriangleIndex)(void*,int);
void(btMultiBody_delete)(void*);
void*(btSoftBody_getTetras)(void*);
void(btHeightfieldTerrainShape_setUseZigzagSubdivision)(void*);
void(btCollisionAlgorithmCreateFunc_delete)(void*);
float(btSoftBody_Config_getKDF)(void*);
void(btTriangle_setPartId)(void*,int);
void(btSliderConstraint_setDampingDirLin)(void*,float);
void*(btCollisionWorld_RayResultCallbackWrapper_new)(void*,void*);
void(btSliderConstraint_setUpperLinLimit)(void*,float);
void*(btWorldImporter_createCylinderShapeZ)(void*,float,float);
int(btTriangle_getPartId)(void*);
float(btRigidBody_computeAngularImpulseDenominator)(void*,const void*);
float(btContactSolverInfoData_getFrictionCfm)(void*);
void(btSoftBody_Impulse_getDrift)(void*,void*);
void*(btGhostPairCallback_new)();
void(btGeneric6DofSpring2Constraint_setLinearUpperLimit)(void*,const void*);
float(btConeTwistConstraint_getBiasFactor)(void*);
void(btSoftBody_Body_activate)(void*);
int(btBroadphaseProxy_getCollisionFilterGroup)(void*);
void(btManifoldPoint_setContactMotion2)(void*,float);
void(btConvexSeparatingDistanceUtil_updateSeparatingDistance)(void*,const void*,const void*);
void(btConvexSeparatingDistanceUtil_initSeparatingDistance)(void*,const void*,float,const void*,const void*);
float(btConvexSeparatingDistanceUtil_getConservativeSeparatingDistance)(void*);
void(btMultiBodySliderConstraint_getFrameInB)(void*,void*);
void*(btTranslationalLimitMotor_new2)(const void*);
void(btRigidBody_setGravity)(void*,const void*);
void(btAlignedObjectArray_btIndexedMesh_resizeNoInitialize)(void*,int);
void(btDbvtAabbMm_Expand)(void*,const void*);
void(btSoftBody_Body_invWorldInertia)(void*,void*);
void(btSoftBody_Joint_Terminate)(void*,float);
void*(btSoftSoftCollisionAlgorithm_new2)(void*,const void*,const void*,const void*);
void(btTransformUtil_calculateVelocityQuaternion)(const void*,const void*,const void*,const void*,float,void*,void*);
void*(btAlignedObjectArray_btSoftBody_Note_at)(void*,int);
void(btDbvt_optimizeBottomUp)(void*);
void(btStridingMeshInterface_setPremadeAabb)(void*,const void*,const void*);
void(btSoftBody_Body_applyVAImpulse)(void*,const void*);
void(btSoftBody_Cluster_getLocii)(void*,void*);
void(btBU_Simplex1to4_reset)(void*);
bool(btBulletWorldImporter_convertAllObjects)(void*,void*);
void(btSoftBody_Pose_getCom)(void*,void*);
void(btSliderConstraint_getAncorInB)(void*,void*);
int(btGImpactQuantizedBvh_getRightNode)(void*,int);
void*(btBU_Simplex1to4_new2)(const void*);
void(btStridingMeshInterface_delete)(void*);
void(btStridingMeshInterface_setScaling)(void*,const void*);
void(btMultibodyLink_setJointDamping)(void*,float);
void(btSoftBody_addVelocity2)(void*,const void*,int);
void(btTransformUtil_calculateDiffAxisAngleQuaternion)(const void*,const void*,void*,float*);
const char*(btStridingMeshInterface_serialize)(void*,void*,void*);
void*(btConvexHullShape_new4)(const float*,int,int);
void(btStridingMeshInterface_preallocateVertices)(void*,int);
void(btSoftBody_Element_delete)(void*);
int(btUnionFind_find2)(void*,int);
int(btPersistentManifold_getCacheEntry)(void*,const void*);
bool(btStridingMeshInterface_hasPremadeAabb)(void*);
int(btGImpactMeshShape_getMeshPartCount)(void*);
void(btStridingMeshInterface_getScaling)(void*,void*);
int(btTypedConstraint_btConstraintInfo2_getNumIterations)(void*);
void(btStridingMeshInterface_getLockedVertexIndexBase)(void*,unsigned char**,int*,int*,int*,unsigned char**,int*,int*,int*,int);
int(btPersistentManifold_getCompanionIdB)(void*);
void(btDiscreteDynamicsWorld_debugDrawConstraint)(void*,void*);
void(btStaticPlaneShape_getPlaneNormal)(void*,void*);
float(btStaticPlaneShape_getPlaneConstant)(void*);
float(btSliderConstraint_getRestitutionDirLin)(void*);
bool(btSoftBody_Pose_getBvolume)(void*);
int(btSoftBody_Node_getBattach)(void*);
void(btDefaultCollisionConstructionInfo_setPersistentManifoldPool)(void*,void*);
void*(btSphereTriangleCollisionAlgorithm_new2)(const void*);
void(btSoftBody_appendNote2)(void*,const char*,const void*,void*);
float(btSoftBody_sCti_getOffset)(void*);
void*(btBroadphaseInterface_getOverlappingPairCache)(void*);
void(btCollisionWorld_updateAabbs)(void*);
void*(btSphereSphereCollisionAlgorithm_CreateFunc_new)();
int(btPrimitiveManagerBase_get_primitive_count)(void*);
int(btCollisionWorld_getNumCollisionObjects)(void*);
int(btSoftBody_generateBendingConstraints)(void*,int,void*);
void(btHingeConstraint_setLimit4)(void*,float,float,float,float,float);
void(btConvexPolyhedron_setExtents)(void*,const void*);
void(btConvexTriangleCallback_getAabbMin)(void*,void*);
void(btDbvtProxy_setLeaf)(void*,void*);
float(btSphereBoxCollisionAlgorithm_getSpherePenetration)(void*,const void*,const void*,void*,void*);
void(btSoftBody_Joint_Specs_delete)(void*);
float(btSoftBody_Joint_getSplit)(void*);
void(btSoftBody_appendTetra)(void*,int,void*);
void*(btSphereBoxCollisionAlgorithm_new)(void*,const void*,const void*,const void*,bool);
void(btMinkowskiSumShape_setTransformA)(void*,const void*);
void*(btSphereBoxCollisionAlgorithm_CreateFunc_new)();
const void*(btMultibodyLink_getUserPtr)(void*);
void(btHingeConstraint_getAFrame)(void*,void*);
void(btSparseSdf3_Reset)(void*);
int(btSoftBody_Config_getCollisions)(void*);
void(btConvexInternalShape_getImplicitShapeDimensions)(void*,void*);
void(btSoftBody_initializeClusters)(void*);
int(btSparseSdf3_RemoveReferences)(void*,void*);
void(btContactSolverInfoData_setMaxGyroscopicForce)(void*,float);
void*(btWorldImporter_getBvhByIndex)(void*,int);
void(btDbvt_sStkNPS_setNode)(void*,const void*);
void*(btSoftSoftCollisionAlgorithm_new)(const void*);
void*(btSoftSoftCollisionAlgorithm_CreateFunc_new)();
void(btSoftRigidDynamicsWorld_setDrawFlags)(void*,int);
float(btSoftBody_Node_getArea)(void*);
void(btMultibodyLink_getCachedRotParentToThis)(void*,void*);
bool(btConeTwistConstraint_isMaxMotorImpulseNormalized)(void*);
void(btAlignedObjectArray_btSoftBody_MaterialPtr_resizeNoInitialize)(void*,int);
float(btHingeConstraint_getMotorTargetVelocity)(void*);
void(btCharacterControllerInterface_warp)(void*,const void*);
void*(btSoftRigidCollisionAlgorithm_new)(void*,const void*,const void*,const void*,bool);
void(btCollisionWorld_removeCollisionObject)(void*,void*);
int(btSoftRigidDynamicsWorld_getDrawFlags)(void*);
bool(btHingeConstraint_getUseReferenceFrameA)(void*);
void*(btPairSet_new)();
void(btMultibodyLink_setAppliedTorque)(void*,const void*);
void(btSoftRigidDynamicsWorld_addSoftBody2)(void*,void*,int);
void(btRigidBody_applyCentralImpulse)(void*,const void*);
void(btGeneric6DofSpring2Constraint_setServoTarget)(void*,int,float);
void*(btSoftRigidDynamicsWorld_getSoftBodyArray)(void*);
void*(btRaycastVehicle_btVehicleTuning_new)();
float(btManifoldPoint_getContactMotion1)(void*);
void(btCollisionWorld_ClosestRayResultCallback_setRayFromWorld)(void*,const void*);
void(btGImpactShapeInterface_getBulletTriangle)(void*,int,void*);
void(btSoftBodySolverOutput_delete)(void*);
void(btSoftBodySolver_setNumberOfVelocityIterations)(void*,int);
void(btGeneric6DofSpring2Constraint_setServo)(void*,int,bool);
void(btSoftBodySolver_setNumberOfPositionIterations)(void*,int);
void(btConeShape_setConeUpIndex)(void*,int);
void(btSoftBodySolver_predictMotion)(void*,float);
void*(btCollisionWorld_AllHitsRayResultCallback_getHitNormalWorld)(void*);
int(btSoftBodySolver_getNumberOfVelocityIterations)(void*);
void*(btWorldImporter_createCollisionObject)(void*,const void*,void*,const char*);
void(btGeneric6DofConstraint_getFrameOffsetB)(void*,void*);
void(btRigidBody_btRigidBodyConstructionInfo_getLocalInertia)(void*,void*);
void*(btCollisionObject_getCollisionShape)(void*);
void(btConeTwistConstraint_getAFrame)(void*,void*);
bool(btSoftBodySolver_checkInitialized)(void*);
bool(btCollisionWorld_RayResultCallbackWrapper_needsCollision)(void*,void*);
int(btRotationalLimitMotor_testLimitValue)(void*,float);
void(btContactSolverInfoData_setFrictionErp)(void*,float);
void(btSoftBodyHelpers_DrawNodeTree)(void*,void*,int,int);
void(btSoftBodyHelpers_DrawInfos)(void*,void*,bool,bool,bool);
int(btGjkPairDetector_getLastUsedMethod)(void*);
int(btMultiBody_getParent)(void*,int);
void(btTriangle_getVertex1)(void*,void*);
void(btDiscreteDynamicsWorld_setApplySpeculativeContactRestitution)(void*,bool);
float(btRotationalLimitMotor_getMaxMotorForce)(void*);
void(btSoftBodyHelpers_DrawClusterTree)(void*,void*,int,int);
void*(btSoftBodyHelpers_CreatePatchUV)(void*,const void*,const void*,const void*,const void*,int,int,int,bool,float*);
void(btAxisSweep3_setOverlappingPairUserCallback)(void*,void*);
void(btSoftBody_RContact_getC1)(void*,void*);
void*(btSoftBodyConcaveCollisionAlgorithm_new)(const void*,const void*,const void*,bool);
int(btSoftBody_getTetraVertexNormalData)(void*,float*);
void(btPolyhedralConvexShape_getEdge)(void*,int,void*,void*);
float(btGeneric6DofSpringConstraint_getStiffness)(void*,int);
void(btCollisionShape_delete)(void*);
void*(btDbvt_array_at)(void*,int);
void(btCollisionShape_getLocalScaling)(void*,void*);
void(btSoftBody_ImplicitFn_delete)(void*);
void(btWheelInfo_getWheelDirectionCS)(void*,void*);
void(btGImpactMeshShapePart_TrimeshPrimitiveManager_setPart)(void*,int);
void(btMultibodyLink_setAppliedConstraintTorque)(void*,const void*);
void*(btMultiBodySliderConstraint_new2)(void*,int,void*,int,const void*,const void*,const void*,const void*,const void*);
void(btSoftBody_updateNormals)(void*);
void(btSoftBody_Config_setKLF)(void*,float);
void*(btOptimizedBvh_deSerializeInPlace)(void*,unsigned int,bool);
void(btUniversalConstraint_getAnchor)(void*,void*);
void(btTranslationalLimitMotor2_setStopERP)(void*,const void*);
void(btSoftBody_updateBounds)(void*);
void*(btConeTwistConstraint_new)(void*,void*,const void*,const void*);
void(btCollisionWorld_rayTestSingle)(const void*,const void*,void*,const void*,const void*,void*);
float(btSoftBody_Config_getKSRHR_CL)(void*);
void(btSoftBody_translate)(void*,const void*);
void(btSoftBody_transform)(void*,const void*);
void*(btCollisionWorld_getDispatcher)(void*);
void(btCollisionObject_setSpinningFriction)(void*,float);
void(btSoftBody_Joint_Specs_setErp)(void*,float);
void(btSoftBody_solveConstraints)(void*);
void(btSoftBody_solveCommonConstraints)(void**,int,int);
void(btSoftBody_solveClusters2)(void*,float);
bool(btVoronoiSimplexSolver_closestPtPointTriangle)(void*,const void*,const void*,const void*,const void*,void*);
void*(btAlignedObjectArray_btPersistentManifoldPtr_new)();
void(btSoftBody_setVolumeMass)(void*,float);
void(btHingeConstraint_updateRHS)(void*,float);
void(btWheelInfo_RaycastInfo_getContactPointWS)(void*,void*);
bool*(btTranslationalLimitMotor2_getEnableMotor)(void*);
void(btSoftBody_PSolve_Anchors)(void*,float,float);
void*(btMultiBodyFixedConstraint_new)(void*,int,void*,const void*,const void*,const void*,const void*);
void(btSoftBody_setWindVelocity)(void*,const void*);
void*(btConvexPointCloudShape_getUnscaledPoints)(void*);
void(btSoftBody_setVelocity)(void*,const void*);
void(btSoftBody_setTotalMass)(void*,float,bool);
void(btSoftBody_setTag)(void*,void*);
void(btGeneric6DofConstraint_getCalculatedTransformA)(void*,void*);
void(btSoftBody_setSolver)(void*,int);
float(btWheelInfo_getSuspensionStiffness)(void*);
void(btSoftBody_setRestLengthScale)(void*,float);
void(btSoftBody_setPose)(void*,bool,bool);
void(btSoftBody_setMass)(void*,int,float);
void(btSoftBody_setInitialWorldTransform)(void*,const void*);
void(btMultibodyLink_setEVector)(void*,const void*);
void(btSoftBody_scale)(void*,const void*);
void(btSoftBody_rotate)(void*,const void*);
void(btMultiBody_getBaseVel)(void*,void*);
void(btSoftBody_resetLinkRestLengths)(void*);
void(btSoftBody_releaseClusters)(void*);
bool(btOverlapCallback_processOverlap)(void*,void*);
void*(btOptimizedBvh_new)();
void(btSoftBody_releaseCluster)(void*,int);
void(btSoftBody_refine)(void*,void*,float,bool);
void(btVoronoiSimplexSolver_getLastW)(void*,void*);
bool(btSoftBody_rayTest)(void*,const void*,const void*,void*);
void(btSoftBody_PSolve_SContacts)(void*,float,float);
void*(btGImpactMeshShape_new)(void*);
void(btSoftBody_PSolve_RContacts)(void*,float,float);
void(btTranslationalLimitMotor2_setBounce)(void*,const void*);
void(btSoftBody_RayFromToCaster_setRayFrom)(void*,const void*);
void(btSoftBody_prepareClusters)(void*,int);
void*(btSoftBody_LJoint_getRpos)(void*);
void(btSoftBody_pointersToIndices)(void*);
bool(btDbvt_empty)(void*);
void(btMultibodyLink_setJointFeedback)(void*,void*);
const void*(btDbvt_sStkNN_getA)(void*);
void(btSoftBody_integrateMotion)(void*);
int(btBvhTree_getRightNode)(void*,int);
void(btMultiBody_getBaseOmega)(void*,void*);
void(btMultibodyLink_getAppliedTorque)(void*,void*);
float(btContactSolverInfoData_getFriction)(void*);
void(btSoftBody_initDefaults)(void*);
void(btMultiBody_addLinkTorque)(void*,int,const void*);
void(btSoftBody_indicesToPointers)(void*,const int*);
float(btPrimitiveTriangle_getDummy)(void*);
float(btRigidBody_computeImpulseDenominator)(void*,const void*,const void*);
void(btSoftBodyWorldInfo_setGravity)(void*,const void*);
void(btRotationalLimitMotor2_setMotorERP)(void*,float);
bool(btBvhTriangleMeshShape_getOwnsBvh)(void*);
void*(btSoftBody_Joint_Specs_new)();
void*(btSoftBody_getUserIndexMapping)(void*);
float(btSoftBody_getTimeacc)(void*);
int(btTriangleMesh_getNumTriangles)(void*);
void(btSoftBody_appendNote5)(void*,const char*,const void*,const void*,void*,void*,void*,void*);
void*(btSoftBody_getSoftBodySolver)(void*);
void*(btSoftBody_getScontacts)(void*);
void(btDiscreteCollisionDetectorInterface_ClosestPointInput_setMaximumDistanceSquared)(void*,float);
void*(btAlignedObjectArray_btSoftBody_JointPtr_at)(void**,int);
float(btSoftBody_getRestLengthScale)(void*);
void*(btSoftBody_getRcontacts)(void*);
float(btMultiBodySolverConstraint_getAppliedPushImpulse)(void*);
void*(btSoftBody_getPose)(void*);
void(btAxisSweep3_quantize)(void*,unsigned short*,const void*,int);
void*(btSoftBody_getNdbvt)(void*);
void*(btSoftBody_getMaterials)(void*);
float(btSoftBody_getMass)(void*,int);
void*(btSoftBody_getJoints)(void*);
void(btSoftBody_getInitialWorldTransform)(void*,void*);
void*(btSoftBody_getFdbvt)(void*);
void(btBvhTriangleMeshShape_serializeSingleBvh)(void*,void*);
void*(btSoftBody_getCollisionDisabledObjects)(void*);
void*(btGImpactMeshShapePart_new2)(void*,int);
void*(btSoftBody_getClusterConnectivity)(void*);
int(btMultiSphereShape_getSphereCount)(void*);
void(btSimulationIslandManager_buildIslands)(void*,void*,void*);
void(btCharacterControllerInterface_setWalkDirection)(void*,const void*);
void(btMultiBody_addJointTorqueMultiDof)(void*,int,const float*);
void(btSliderConstraint_testAngLimits)(void*);
void*(btBox2dBox2dCollisionAlgorithm_CreateFunc_new)();
void*(btSoftBody_getBounds)(void*);
void(btMultiBody_updateCollisionObjectWorldTransforms)(void*,void*,void*);
bool(btRotationalLimitMotor_isLimited)(void*);
void*(btSoftBody_getAnchors)(void*);
int(btSoftBody_generateClusters2)(void*,int,int);
void(btMultiBodySliderConstraint_getFrameInA)(void*,void*);
void*(btPolarDecomposition_new)(float,unsigned int);
float(btSoftBody_Joint_getErp)(void*);
int(btMultiBodySolverConstraint_getOrgDofIndex)(void*);
int(btSoftBody_generateClusters)(void*,int);
void(btConeTwistConstraint_getInfo2NonVirtual)(void*,void*,const void*,const void*,const void*,const void*);
void(btMultiBodyFixedConstraint_setPivotInA)(void*,const void*);
float(btManifoldPoint_getDistance1)(void*);
float(btConstraintSolver_solveGroup)(void*,void**,int,void**,int,void**,int,const void*,void*,void*);
int(btMultiBodyConstraint_getNumRows)(void*);
int(btCollisionObject_getUserIndex)(void*);
void(btSoftBody_SolverState_setSdt)(void*,float);
void*(btBox2dShape_new2)(float);
bool(btSoftBody_cutLink2)(void*,int,int,float);
void(btSoftBody_clusterVImpulse)(void*,const void*,const void*);
void(btTranslationalLimitMotor2_getMotorERP)(void*,void*);
void(btSoftBody_clusterVAImpulse)(void*,const void*);
void(btRigidBody_btRigidBodyConstructionInfo_setCollisionShape)(void*,void*);
void(btSoftBody_clusterDImpulse)(void*,const void*,const void*);
void(btSoftBody_clusterDCImpulse)(void*,const void*);
void(btSoftBody_clusterDAImpulse)(void*,const void*);
int(btTypedConstraint_btConstraintInfo2_getRowskip)(void*);
void(btSoftBody_clusterCom2)(const void*,void*);
int(btPoint2PointConstraint_getFlags)(void*);
void(btSoftBody_clusterAImpulse)(void*,const void*);
void*(btMultiBodyPoint2Point_new)(void*,int,void*,const void*,const void*);
float(btMultiBody_getJointVel)(void*,int);
void*(btSoftBody_Link_new2)(void*);
void(btSoftBody_cleanupClusters)(void*);
bool(btSoftBody_checkLink2)(void*,int,int);
bool(btSoftBody_checkFace)(void*,int,int,int);
int(btSoftBody_CJoint_getLife)(void*);
void*(btDbvt_sStkCLN_getParent)(void*);
void(btAlignedObjectArray_btIndexedMesh_push_back)(void*,void*);
void(btCollisionWorld_LocalConvexResult_setHitFraction)(void*,float);
void(btGeneric6DofConstraint_setLinearUpperLimit)(void*,const void*);
void(btGImpactCollisionAlgorithm_setFace0)(void*,int);
void(btMultiBody_localPosToWorld)(void*,int,const void*,void*);
void(btRigidBody_btRigidBodyConstructionInfo_setStartWorldTransform)(void*,const void*);
void(btManifoldPoint_setPositionWorldOnA)(void*,const void*);
void*(btGImpactMeshShapePart_TrimeshPrimitiveManager_getMeshInterface)(void*);
void*(btSoftBody_getSst)(void*);
void(btDynamicsWorld_removeAction)(void*,void*);
void(btSoftBody_appendNote4)(void*,const char*,const void*);
void(btPointCollector_setNormalOnBInWorld)(void*,const void*);
void*(btRaycastVehicle_getRigidBody)(void*);
void(btSoftBody_appendNote3)(void*,const char*,const void*,void*);
void(btMultiBody_useRK4Integration)(void*,bool);
float(btManifoldResult_getClosestPointDistanceThreshold)(void*);
void(btGeneric6DofConstraint_setUseSolveConstraintObsolete)(void*,bool);
void(btSoftBody_Joint_Type)(void*);
float*(btMultiBodyConstraint_jacobianB)(void*,int);
void*(btSoftBody_Pose_getWgh)(void*);
void*(btSoftBody_Joint_getBodies)(void*);
float(btSoftBody_Config_getKVC)(void*);
void(btSoftBody_appendLink2)(void*,int,void*);
bool(btCollisionShape_isNonMoving)(void*);
const void*(btSoftBody_sCti_getColObj)(void*);
void(btCollisionObject_getAnisotropicFriction)(void*,void*);
void*(btSortedOverlappingPairCache_new)();
int(btQuantizedBvhTree_getRightNode)(void*,int);
void(btSliderConstraint_setRestitutionDirLin)(void*,float);
float(btWheelInfo_getSuspensionRestLength)(void*);
void(btOverlappingPairCache_sortOverlappingPairs)(void*,void*);
void(btMultiBodyConstraint_updateJacobianSizes)(void*);
bool(btGImpactQuantizedBvh_hasHierarchy)(void*);
void(btAxisSweep3_unQuantize)(void*,void*,void*,void*);
void(btSoftBody_appendLinearJoint2)(void*,const void*);
void*(btSoftBody_AJoint_Specs_new)();
bool(btQuantizedBvh_serialize)(void*,void*,unsigned int,bool);
void*(btSoftBody_Anchor_getNode)(void*);
void(btSoftBody_appendFace)(void*,int,void*);
bool(btBroadphaseProxy_isPolyhedral)(int);
void(btVoronoiSimplexSolver_setEqualVertexThreshold)(void*,float);
float(btManifoldPoint_getContactMotion2)(void*);
void(btSoftBody_appendAngularJoint)(void*,const void*);
void(btSoftBody_appendAnchor2)(void*,int,void*,bool,float);
void(btSoftBody_appendAnchor)(void*,int,void*,const void*,bool,float);
void(btTypedConstraint_btConstraintInfo1_delete)(void*);
void*(btSoftBody_AJoint_Specs_getIcontrol)(void*);
void*(btSphereSphereCollisionAlgorithm_new)(void*,const void*,const void*,const void*);
void(btCompoundShape_calculatePrincipalAxisTransform)(void*,float*,void*,void*);
bool(btGeneric6DofSpring2Constraint_matrixToEulerYXZ)(const void*,void*);
void(btDispatcherInfo_setUseContinuous)(void*,bool);
bool(btConvexPolyhedron_testContainment)(void*);
void*(btSoftBody_new)(void*,int,const float*,const float*);
void(btConvexPolyhedron_getME)(void*,void*);
void(btSoftBody_Tetra_setLeaf)(void*,void*);
void(btSoftBody_Tetra_setC2)(void*,float);
void(btSoftBody_Tetra_setC1)(void*,float);
void**(btSoftBody_Tetra_getN)(void*);
void*(btBoxShape_new3)(float,float,float);
void(btCollisionObject_removeCustomDebugColor)(void*);
bool(btDbvt_ICollide_AllLeaves)(void*,const void*);
void(btSoftBody_Node_getX)(void*,void*);
void(btConvexInternalAabbCachingShape_recalcLocalAabb)(void*);
void(btTriangleInfoMap_delete)(void*);
void*(btSoftBody_Tetra_getC0)(void*);
void(btDiscreteCollisionDetectorInterface_getClosestPoints)(void*,const void*,void*,void*,bool);
void(btSoftBody_sRayCast_delete)(void*);
void(btSoftBody_sRayCast_setIndex)(void*,int);
void(btRigidBody_computeGyroscopicImpulseImplicit_Body)(void*,float,void*);
void(btSoftBody_sRayCast_setFraction)(void*,float);
void(btRigidBody_getTotalTorque)(void*,void*);
void(btSoftBody_sRayCast_setFeature)(void*,int);
void(btSoftBody_sRayCast_setBody)(void*,void*);
void*(btMultimaterialTriangleMeshShape_new2)(void*,bool,const void*,const void*,bool);
int(btSoftBody_sRayCast_getIndex)(void*);
float(btSoftBody_sRayCast_getFraction)(void*);
int(btMLCPSolver_getNumFallbacks)(void*);
void*(btMultiBodySolverConstraint_getMultiBodyA)(void*);
void(btSoftBody_SolverState_setVelmrg)(void*,float);
void(btWheelInfoConstructionInfo_getWheelDirectionCS)(void*,void*);
bool(btDispatcherInfo_getEnableSatConvex)(void*);
void(btSoftBody_dampClusters)(void*);
float*(btFace_getPlane)(void*);
void**(btDbvtProxy_getLinks)(void*);
void*(btDbvtNode_getParent)(void*);
void(btSoftBody_SolverState_setRadmrg)(void*,float);
void(btSoftBody_SolverState_setIsdt)(void*,float);
float(btSoftBody_SolverState_getVelmrg)(void*);
void(btRigidBody_getLinearFactor)(void*,void*);
int(btAlignedObjectArray_btCollisionObjectPtr_findLinearSearch2)(void*,void*);
float(btSoftBody_SolverState_getUpdmrg)(void*);
float(btSoftBody_SolverState_getSdt)(void*);
void(btCollisionObject_setWorldTransform)(void*,const void*);
float(btSoftBody_SolverState_getRadmrg)(void*);
float(btSoftBody_SolverState_getIsdt)(void*);
void(btBoxShape_getHalfExtentsWithoutMargin)(void*,void*);
void(btSoftBody_sCti_delete)(void*);
void(btSoftBody_sCti_setOffset)(void*,float);
void(btSoftBody_sCti_setNormal)(void*,const void*);
void(btSoftBody_sCti_setColObj)(void*,const void*);
void*(btSphereTriangleCollisionAlgorithm_CreateFunc_new)();
void(btSoftBody_Body_delete)(void*);
void*(btSoftBody_sCti_new)();
void(btSoftBody_Config_setKDG)(void*,float);
void(btSoftBody_SContact_setWeights)(void*,const void*);
void(btSoftBody_SContact_setNormal)(void*,const void*);
void(btSoftBody_SContact_setNode)(void*,void*);
void(btSoftBody_SContact_setMargin)(void*,float);
void(btSoftBody_SContact_setFriction)(void*,float);
void(btSoftBody_SContact_setFace)(void*,void*);
void(btCharacterControllerInterface_playerStep)(void*,void*,float);
void(btSoftBody_AJoint_IControl_delete)(void*);
void*(btBroadphaseProxy_getClientObject)(void*);
void(btSoftBody_SContact_getNormal)(void*,void*);
void(btChunk_setOldPtr)(void*,void*);
void(btIDebugDraw_delete)(void*);
void*(btSoftBody_Joint_getRefs)(void*);
void*(btSoftBody_SContact_getNode)(void*);
void(btCollisionWorld_ClosestRayResultCallback_setRayToWorld)(void*,const void*);
float(btSoftBody_SContact_getFriction)(void*);
float*(btSoftBody_SContact_getCfm)(void*);
float(btMultiBodySolverConstraint_getLowerLimit)(void*);
void*(btGImpactMeshShape_getMeshPart)(void*,int);
void(btSoftBody_Link_setC0)(void*,float);
void(btSoftBody_RContact_setNode)(void*,void*);
void(btRotationalLimitMotor_setCurrentPosition)(void*,float);
void(btSoftBody_Pose_getAqq)(void*,void*);
bool(btTypedConstraint_needsFeedback)(void*);
void(btSoftBody_RContact_setC3)(void*,float);
void(btSoftBody_RContact_setC2)(void*,float);
void(btSoftBody_RContact_setC1)(void*,const void*);
void(btSoftBody_RContact_setC0)(void*,const void*);
int(btMultiBodyDynamicsWorld_getNumMultiBodyConstraints)(void*);
int(btCapsuleShape_getUpAxis)(void*);
float(btContactSolverInfoData_getLinearSlop)(void*);
float(btManifoldResult_calculateCombinedFriction)(const void*,const void*);
void*(btSoftBody_RContact_getNode)(void*);
void*(btCompoundShape_getDynamicAabbTree)(void*);
void*(btCollisionAlgorithmConstructionInfo_getDispatcher1)(void*);
float(btSoftBody_RContact_getC3)(void*);
void(btGhostObject_removeOverlappingObjectInternal)(void*,void*,void*,void*);
void*(btConvexConvexAlgorithm_CreateFunc_getPdSolver)(void*);
float(btSoftBody_RContact_getC2)(void*);
void(btGImpactCollisionAlgorithm_gimpact_vs_shape)(void*,const void*,const void*,const void*,const void*,bool);
void(btDbvt_sStkCLN_delete)(void*);
void(btSoftBody_RayFromToCaster_setTests)(void*,int);
void(btVehicleRaycaster_btVehicleRaycasterResult_delete)(void*);
const char*(btSoftBody_Note_getText)(void*);
bool(btDispatcher_needsCollision)(void*,const void*,const void*);
void(btSoftBody_PSolve_Links)(void*,float,float);
bool(btBroadphaseProxy_isCompound)(int);
void(btSoftBody_RayFromToCaster_setMint)(void*,float);
void(btSoftBody_RayFromToCaster_setFace)(void*,void*);
void*(btCollisionWorld_ContactResultCallbackWrapper_new)(void*,void*);
float(btSoftBody_RayFromToCaster_rayFromToTriangle2)(const void*,const void*,const void*,const void*,const void*,const void*,float);
void(btSoftBody_RayFromToCaster_getRayTo)(void*,void*);
void*(btMultiBodyConstraint_getMultiBodyA)(void*);
void*(btSoftBody_CJoint_getRpos)(void*);
void(btBroadphasePair_delete)(void*);
void(btRotationalLimitMotor_setStopERP)(void*,float);
const void*(btPersistentManifold_getBody0)(void*);
float(btTriangleMesh_getWeldingThreshold)(void*);
void(btSoftBody_Element_setTag)(void*,void*);
void(btSoftBody_Pose_setVolume)(void*,float);
void(btMultiBody_addLinkConstraintForce)(void*,int,const void*);
void(btAxisSweep3_updateHandle)(void*,unsigned short,const void*,const void*,void*);
void(btSoftBody_Pose_setRot)(void*,const void*);
void(btSoftBody_Pose_setCom)(void*,const void*);
void(btHingeConstraint_setUseFrameOffset)(void*,bool);
void(btSoftBody_Pose_setBvolume)(void*,bool);
void*(btSoftBody_AJoint_IControl_Default)();
void(btSoftBody_Pose_setBframe)(void*,bool);
void(btSoftBody_Pose_setAqq)(void*,const void*);
void*(btCollisionWorld_getDebugDrawer)(void*);
bool(btAABB_overlapping_trans_cache)(void*,const void*,const void*,bool);
void(btSoftBody_Pose_getRot)(void*,void*);
void(btBU_Simplex1to4_addVertex)(void*,const void*);
void(btMultibodyLink_getInertiaLocal)(void*,void*);
bool(btSoftBody_Pose_getBframe)(void*);
void(btKinematicCharacterController_setLinearDamping)(void*,float);
void(btMultibodyLink_setAxisBottom2)(void*,int,const void*);
void(btSoftBody_Note_setText)(void*,const char*);
void(btSoftBody_Note_setRank)(void*,int);
void(btSoftBody_Note_setOffset)(void*,const void*);
void(btSoftBody_RayFromToCaster_setRayNormalizedDirection)(void*,const void*);
void(btCollisionDispatcher_defaultNearCallback)(void*,void*,const void*);
void(btCollisionWorld_RayResultCallback_setCollisionObject)(void*,const void*);
void*(btSoftBody_Face_getLeaf)(void*);
void(btSoftBody_Note_getOffset)(void*,void*);
void**(btSoftBody_Note_getNodes)(void*);
float(btCollisionWorld_ConvexResultCallback_addSingleResult)(void*,void*,bool);
void(btMLCPSolver_setNumFallbacks)(void*,int);
bool(btTranslationalLimitMotor_isLimited)(void*,int);
void(btGjkPairDetector_setPenetrationDepthSolver)(void*,void*);
float(btMultiBody_getMaxCoordinateVelocity)(void*);
void(btSoftBody_Node_setV)(void*,const void*);
void(btSoftBody_Node_setLeaf)(void*,void*);
void(btKinematicCharacterController_setStepHeight)(void*,float);
void(btSoftBody_Node_setIm)(void*,float);
void(btSoftBody_Node_setF)(void*,const void*);
void(btSoftBody_Node_setBattach)(void*,int);
void(btSoftBody_Node_setArea)(void*,float);
void*(btBroadphasePair_new2)(const void*);
float(btConeTwistConstraint_getTwistAngle)(void*);
void(btCollisionWorld_ClosestConvexResultCallback_getConvexToWorld)(void*,void*);
float(btSliderConstraint_getDampingLimAng)(void*);
void(btSoftBody_Node_getV)(void*,void*);
void(btSoftBody_Node_getQ)(void*,void*);
void(btGearConstraint_setAxisA)(void*,void*);
void(btSliderConstraint_setSoftnessLimLin)(void*,float);
void(btSliderConstraint_setLowerLinLimit)(void*,float);
void(btSoftBody_Node_getF)(void*,void*);
void*(btSoftRigidDynamicsWorld_getWorldInfo)(void*);
void(btSoftBody_Material_setKVST)(void*,float);
void(btSoftBody_Material_setKLST)(void*,float);
void(btSoftBody_Material_setKAST)(void*,float);
void(btSoftBody_Material_setFlags)(void*,int);
float(btSoftBody_Material_getKVST)(void*);
int(btSoftBody_Material_getFlags)(void*);
void(btSoftBody_predictMotion)(void*,float);
void(btSoftBody_LJoint_Specs_setPosition)(void*,const void*);
void(btBvhTriangleMeshShape_refitTree)(void*,const void*,const void*);
void(btAABB_setMin)(void*,const void*);
void(btSoftBody_LJoint_Specs_getPosition)(void*,void*);
void*(btSoftBody_LJoint_Specs_new)();
float(btContactSolverInfoData_getErp2)(void*);
void*(btCylinderShape_new2)(float,float,float);
void*(btGeneric6DofConstraint_getTranslationalLimitMotor)(void*);
void(btSoftBody_Link_setRl)(void*,float);
void(btSoftBody_Link_setC3)(void*,const void*);
void(btGeneric6DofConstraint_setUseLinearReferenceFrameA)(void*,bool);
void(btSoftBody_Link_setC2)(void*,float);
void(btSoftBody_Link_setC1)(void*,float);
void(btSoftBody_RContact_delete)(void*);
void*(btBox2dShape_new3)(float,float,float);
void(btSoftBody_Link_setBbending)(void*,int);
void**(btSoftBody_Link_getN)(void*);
void(btCollisionWorld_ClosestConvexResultCallback_getConvexFromWorld)(void*,void*);
void(btDiscreteDynamicsWorld_setNumTasks)(void*,int);
bool(btDiscreteDynamicsWorld_getSynchronizeAllMotionStates)(void*);
void(btRigidBody_getAabb)(void*,void*,void*);
float(btSoftBody_Link_getC1)(void*);
float(btSoftBody_Link_getC0)(void*);
float(btConvexCast_CastResult_getAllowedPenetration)(void*);
int(btContactSolverInfoData_getRestingContactRestitutionThreshold)(void*);
float(btCollisionWorld_LocalRayResult_getHitFraction)(void*);
void*(btQuantizedBvhTree_new)();
bool(btGeneric6DofSpring2Constraint_matrixToEulerZYX)(const void*,void*);
void*(btCompoundCollisionAlgorithm_CreateFunc_new)();
void*(btSoftBody_Link_new)();
void*(btBulletXmlWorldImporter_new)(void*);
int(btMultibodyLink_getPosVarCount)(void*);
float(btAngularLimit_getSign)(void*);
void*(btSoftBody_appendMaterial)(void*);
void(btSoftBody_Joint_Solve)(void*,float,float);
void(btCollisionObjectWrapper_getWorldTransform)(void*,void*);
void(btMultiBodyConstraint_debugDraw)(void*,void*);
void(btWheelInfo_updateWheel)(void*,const void*,void*);
void(btSoftBody_Joint_setSdrift)(void*,const void*);
void(btSoftBody_Joint_setMassmatrix)(void*,const void*);
void(btMultiBody_fillConstraintJacobianMultiDof)(void*,int,const void*,const void*,const void*,float*,void*,void*,void*);
const void*(btCollisionWorld_LocalRayResult_getCollisionObject)(void*);
void(btSoftBody_Joint_setErp)(void*,float);
void(btSoftBody_Joint_setDrift)(void*,const void*);
void(btGeneric6DofSpring2Constraint_calculateTransforms2)(void*);
void(btSoftBody_Joint_setDelete)(void*,bool);
void(btSoftBody_Joint_setCfm)(void*,float);
void(btSoftBody_Joint_Prepare)(void*,float,int);
bool(btSphereBoxCollisionAlgorithm_getSphereDistance)(void*,const void*,void*,void*,float*,const void*,float,float);
void(btSoftBody_Joint_getSdrift)(void*,void*);
float(btHingeAccumulatedAngleConstraint_getAccumulatedHingeAngle)(void*);
void(btSoftBody_Anchor_setC2)(void*,float);
float(btSoftBody_Joint_getCfm)(void*);
int(btHingeConstraint_getFlags)(void*);
void(btSoftBody_appendLink3)(void*,void*,void*,void*,bool);
int(btGImpactBvh_getNodeData)(void*,int);
float(btSoftBody_Joint_Specs_getSplit)(void*);
void(btSoftBody_Joint_Specs_setSplit)(void*,float);
bool(btDbvtNode_isinternal)(void*);
void*(btDynamicsWorld_getWorldUserInfo)(void*);
void(btSoftBody_staticSolve)(void*,int);
void(btAlignedObjectArray_btVector3_push_back2)(void*,const void*);
void*(btWorldImporter_createPoint2PointConstraint2)(void*,void*,void*,const void*,const void*);
void*(btCollisionWorld_getDispatchInfo)(void*);
void(btAngularLimit_delete)(void*);
void(btSoftBody_Impulse_setVelocity)(void*,const void*);
void(btGeneric6DofConstraint_setUseFrameOffset)(void*,bool);
void*(btMultiBodyJointLimitConstraint_new)(void*,int,float,float);
void(btSoftBody_Impulse_setDrift)(void*,const void*);
void(btSoftBody_Impulse_setAsVelocity)(void*,int);
void*(btWorldImporter_createRigidBody)(void*,bool,float,const void*,void*,const char*);
void*(btOverlappingPairCallback_addOverlappingPair)(void*,void*,void*);
void(btTranslationalLimitMotor2_getLowerLimit)(void*,void*);
void(btDbvt_optimizeIncremental)(void*,int);
const void*(btGImpactShapeInterface_getBoxSet)(void*);
void*(btBoxShape_new2)(float);
void(btGImpactCollisionAlgorithm_registerAlgorithm)(void*);
float(btDispatcherInfo_getTimeOfImpact)(void*);
void*(btConvexCast_CastResult_getDebugDrawer)(void*);
void(btMultibodyLink_setCfgOffset)(void*,int);
void*(btSoftBody_Impulse_new)();
int(btSoftBody_getFaceVertexNormalData)(void*,float*);
float(btSoftBody_ImplicitFn_Eval)(void*,const void*);
void*(btConvexConcaveCollisionAlgorithm_CreateFunc_new)();
void(btSoftBody_Feature_setMaterial)(void*,void*);
int(btBvhTree_getEscapeNodeIndex)(void*,int);
void(btCollisionWorld_updateSingleAabb)(void*,void*);
void*(btDbvtNode_new)();
void(btConvexHullShape_optimizeConvexHull)(void*);
void(btSoftBody_Face_setRa)(void*,float);
bool(btTriangleMesh_getUse32bitIndices)(void*);
void(btSoftBody_Face_setLeaf)(void*,void*);
void(btOptimizedBvh_updateBvhNodes)(void*,void*,int,int,int);
void(btCollisionShape_getAnisotropicRollingFrictionDirection)(void*,void*);
int(btGImpactQuantizedBvh_getLeftNode)(void*,int);
float(btSoftBody_Face_getRa)(void*);
void(btSoftBody_Face_getNormal)(void*,void*);
void**(btSoftBody_Face_getN)(void*);
void(btPairSet_push_pair)(void*,int,int);
void*(btSoftBody_RayFromToCaster_new)(const void*,const void*,float);
void(btSoftBody_Body_xform)(void*,void*);
void(btMultibodyLink_getAppliedForce)(void*,void*);
void(btSoftBody_Config_setViterations)(void*,int);
void(btSoftBody_Config_setTimescale)(void*,float);
void(btSoftBody_Config_setPiterations)(void*,int);
void*(btGhostObject_new)();
void(btSoftBody_Config_setMaxvolume)(void*,float);
void(btSoftBody_Config_setKVCF)(void*,float);
void(btSoftBody_Config_setKVC)(void*,float);
void(btBroadphasePair_setPProxy1)(void*,void*);
void(btSoftBody_Config_setKSSHR_CL)(void*,float);
void(btRaycastVehicle_updateWheelTransformsWS2)(void*,void*,bool);
void(btSoftBody_Config_setKSS_SPLT_CL)(void*,float);
void(btMultibodyLink_setPosVarCount)(void*,int);
void(btGImpactMeshShapePart_TrimeshPrimitiveManager_setIndexstride)(void*,int);
int(btCollisionShape_getUserIndex)(void*);
void(btMultiBodyFixedConstraint_setFrameInB)(void*,const void*);
float(btSoftBody_Config_getKCHR)(void*);
void(btMultiBody_setCanSleep)(void*,bool);
int(btGhostObject_getNumOverlappingObjects)(void*);
void(btSoftBody_Config_setKSRHR_CL)(void*,float);
void(btSoftBody_Config_setKSR_SPLT_CL)(void*,float);
void(btConvexPolyhedron_getExtents)(void*,void*);
void(btSliderConstraint_setSoftnessDirLin)(void*,float);
void(btTranslationalLimitMotor2_getMaxMotorForce)(void*,void*);
void*(btConvexPointCloudShape_new2)(void*,int,const void*,bool);
void(btSoftBody_Config_setKSKHR_CL)(void*,float);
void(btRaycastVehicle_btVehicleTuning_delete)(void*);
const char*(btCollisionShape_getName)(void*);
void(btSoftBody_Config_setKSK_SPLT_CL)(void*,float);
void(btSoftBody_Config_setKSHR)(void*,float);
void*(btDynamicsWorld_getSolverInfo)(void*);
void(btSoftBody_Config_setKPR)(void*,float);
void(btSoftBody_Config_setKMT)(void*,float);
void(btSoftBody_updateLinkConstants)(void*);
void*(btSoftBody_Cluster_getMasses)(void*);
float*(btTypedConstraint_btConstraintInfo2_getConstraintError)(void*);
void(btSoftBody_Config_setKDP)(void*,float);
void*(btContinuousConvexCollision_new2)(const void*,const void*);
void(btSoftBody_SContact_delete)(void*);
void(btSoftBody_Config_setKDF)(void*,float);
void(btSoftBody_Config_setKCHR)(void*,float);
void(btBvhTree_build_tree)(void*,void*);
void(btSoftBody_Config_setKAHR)(void*,float);
void(btSoftBody_Config_setDiterations)(void*,int);
int(btBvhTree_getNodeCount)(void*);
void(btSoftBody_Config_setCollisions)(void*,int);
void(btDbvt_setOpath)(void*,unsigned int);
void(btSoftBody_Config_setCiterations)(void*,int);
void(btSoftBody_Config_setAeromodel)(void*,int);
bool(btCharacterControllerInterface_canJump)(void*);
void*(btSoftBody_Config_getVsequence)(void*);
int(btSoftBody_Config_getViterations)(void*);
float(btConeTwistConstraint_getFixThresh)(void*);
void*(btSoftBody_Config_getPsequence)(void*);
int(btSoftBody_Config_getPiterations)(void*);
float(btSoftBody_Config_getKVCF)(void*);
void*(btSoftBody_Cluster_getNodes)(void*);
float(btSoftBody_Config_getKSSHR_CL)(void*);
void(btMultiBody_setBaseCollider)(void*,void*);
void(btVoronoiSimplexSolver_setCachedValidClosest)(void*,bool);
void*(btSoftBody_upcast)(void*);
float(btSoftBody_Config_getKSR_SPLT_CL)(void*);
void(btDbvt_IClone_delete)(void*);
float(btSoftBody_Config_getKSKHR_CL)(void*);
float(btSoftBody_Config_getKSHR)(void*);
float(btSoftBody_Config_getKMT)(void*);
void(btDefaultCollisionConfiguration_setConvexConvexMultipointIterations)(void*,int,int);
float(btSoftBody_Config_getKLF)(void*);
const void*(btDbvt_sStkNPS_getNode)(void*);
void(btManifoldPoint_getPositionWorldOnA)(void*,void*);
float(btSoftBody_Config_getKKHR)(void*);
void(btDefaultCollisionConstructionInfo_setCustomCollisionAlgorithmMaxElementSize)(void*,int);
float(btSoftBody_Config_getKAHR)(void*);
int(btSoftBody_Config_getDiterations)(void*);
void(btBox2dShape_getHalfExtentsWithMargin)(void*,void*);
int*(btTranslationalLimitMotor2_getCurrentLimit)(void*);
void(btSoftBody_Cluster_setSelfCollisionImpulseFactor)(void*,float);
void(btSoftBody_Cluster_setNvimpulses)(void*,int);
void(btSoftBody_Cluster_setNdimpulses)(void*,int);
void*(btContactConstraint_getContactManifold)(void*);
float(btVoronoiSimplexSolver_getEqualVertexThreshold)(void*);
void*(btDbvtAabbMm_FromPoints2)(const void*,int);
void(btDispatcher_dispatchAllCollisionPairs)(void*,void*,const void*,void*);
void(btSoftBody_Cluster_setMaxSelfCollisionImpulse)(void*,float);
void(btSoftBody_Cluster_setMatching)(void*,float);
void(btConvexShape_getPreferredPenetrationDirection)(void*,int,void*);
void(btRotationalLimitMotor2_setStopCFM)(void*,float);
void*(btDispatcherInfo_getDebugDraw)(void*);
void(btRigidBody_getAngularVelocity)(void*,void*);
void(btSoftBody_Cluster_setLv)(void*,const void*);
void*(btMultiBodySolverConstraint_getOriginalContactPoint)(void*);
void(btSoftBody_Cluster_setLocii)(void*,const void*);
void(btSoftBody_Cluster_setLeaf)(void*,void*);
void(btSoftBody_Cluster_setLdamping)(void*,float);
void(btManifoldPoint_setContactPointFlags)(void*,int);
void(btCollisionObject_setIgnoreCollisionCheck)(void*,const void*,bool);
void(btSoftBody_Cluster_setInvwi)(void*,const void*);
void(btSoftBody_Cluster_setImass)(void*,float);
void(btSoftBody_Cluster_setIdmass)(void*,float);
void(btGjkPairDetector_setCachedSeparatingAxis)(void*,const void*);
void(btTriangleInfoMap_setZeroAreaThreshold)(void*,float);
float(btDispatcherInfo_getAllowedCcdPenetration)(void*);
void(btVector3_array_set)(void*,int,const void*);
void(btDiscreteCollisionDetectorInterface_ClosestPointInput_getTransformA)(void*,void*);
void(btSoftBodySolver_updateSoftBodies)(void*);
void(btSoftBody_Cluster_setClusterIndex)(void*,int);
void(btSoftBody_Cluster_setAv)(void*,const void*);
void(btTranslationalLimitMotor_getLowerLimit)(void*,void*);
void(btSoftBody_Cluster_setAdamping)(void*,float);
float(btContactSolverInfoData_getSplitImpulseTurnErp)(void*);
void*(btSoftBody_Cluster_getVimpulses)(void*);
void(btManifoldPoint_setCombinedContactDamping1)(void*,float);
int(btSoftBody_Cluster_getNvimpulses)(void*);
float(btAngularLimit_getHigh)(void*);
float(btSoftBody_Cluster_getNdamping)(void*);
float(btSoftBody_Cluster_getMaxSelfCollisionImpulse)(void*);
int(btAlignedObjectArray_btSoftBody_Face_size)(void*);
void*(btWorldImporter_getRigidBodyByName)(void*,const char*);
void(btSoftBody_Cluster_getLv)(void*,void*);
void(btBroadphaseInterface_delete)(void*);
void(btCompoundShape_updateChildTransform)(void*,int,const void*,bool);
float(btSoftBody_Cluster_getLdamping)(void*);
void(btManifoldPoint_getLateralFrictionDir1)(void*,void*);
float(btMultiBodySolverConstraint_getRhsPenetration)(void*);
void(btMultiBody_checkMotionAndSleepIfRequired)(void*,float);
void(btSoftBody_Cluster_getInvwi)(void*,void*);
void(btHingeAccumulatedAngleConstraint_setAccumulatedHingeAngle)(void*,float);
float(btSoftBody_Cluster_getImass)(void*);
void(btRaycastVehicle_setUserConstraintId)(void*,int);
void(btSoftBody_Cluster_getFramexform)(void*,void*);
void(btAlignedObjectArray_btSoftBody_Node_push_back)(void*,void*);
float(btRaycastVehicle_btVehicleTuning_getSuspensionDamping)(void*);
int(btCollisionWorld_ConvexResultCallback_getCollisionFilterGroup)(void*);
void(btConeTwistConstraint_enableMotor)(void*,bool);
const void*(btGImpactShapeInterface_getLocalBox)(void*);
void(btSoftBody_appendAngularJoint4)(void*,const void*,void*,void*);
void(btConvexConvexAlgorithm_CreateFunc_setMinimumPointsPerturbationThreshold)(void*,int);
void(btRigidBody_btRigidBodyConstructionInfo_setAdditionalAngularDampingThresholdSqr)(void*,float);
void(btMultiBodyFixedConstraint_getFrameInA)(void*,void*);
bool(btSoftBody_Cluster_getCollide)(void*);
void(btGImpactMeshShapePart_getVertex)(void*,int,void*);
void(btConstraintSolver_prepareSolve)(void*,int,int);
void(btGImpactCompoundShape_addChildShape)(void*,const void*,void*);
int(btSoftBody_Cluster_getClusterIndex)(void*);
void(btConvex2dConvex2dAlgorithm_setLowLevelOfDetail)(void*,bool);
void(btSliderConstraint_setRestitutionDirAng)(void*,float);
void(btConeTwistConstraint_setAngularOnly)(void*,bool);
float(btSoftBody_Cluster_getAdamping)(void*);
int(btMaterialProperties_getNumTriangles)(void*);
float(btTranslationalLimitMotor_getDamping)(void*);
float(btWheelInfo_getWheelsDampingCompression)(void*);
void(btSoftBody_CJoint_setLife)(void*,int);
float(btMultiBody_getKineticEnergy)(void*);
void(btSoftBody_CJoint_setFriction)(void*,float);
void(btSoftBody_CJoint_getNormal)(void*,void*);
bool(btBroadphaseProxy_isInfinite)(int);
void(btMultiBodyJointMotor_setVelocityTarget)(void*,float);
const char*(btMultibodyLink_getJointName)(void*);
bool(btSliderConstraint_getPoweredLinMotor)(void*);
void(btSoftBody_Body_velocity)(void*,const void*,void*);
void(btSoftBody_Body_setSoft)(void*,void*);
void(btRaycastVehicle_getChassisWorldTransform)(void*,void*);
void(btSoftBody_Body_setRigid)(void*,void*);
int(btMultiBody_getUserIndex)(void*);
void*(btCollisionWorld_getPairCache)(void*);
void(btSoftBody_Body_setCollisionObject)(void*,const void*);
void(btTransformUtil_integrateTransform)(const void*,const void*,const void*,float,void*);
void(btSimulationIslandManager_storeIslandActivationState)(void*,void*);
float(btSoftBody_Body_invMass)(void*);
const void*(btSoftBody_Body_getCollisionObject)(void*);
void(btSoftBody_Body_applyVImpulse)(void*,const void*,const void*);
void(btTransformUtil_calculateDiffAxisAngle)(const void*,const void*,void*,float*);
void(btSoftBody_Body_applyImpulse)(void*,const void*,const void*);
void(btConeTwistConstraint_calcAngleInfo2)(void*,const void*,const void*,const void*,const void*);
void(btSoftBody_Body_applyDImpulse)(void*,const void*,const void*);
void(btSoftBody_Body_applyDCImpulse)(void*,const void*);
void(btSoftBody_Body_applyDAImpulse)(void*,const void*);
int(btSoftBody_Cluster_getNdimpulses)(void*);
bool*(btTranslationalLimitMotor2_getServoMotor)(void*);
void*(btSoftBody_Body_new3)(void*);
void*(btSoftBody_Body_new2)(const void*);
void(btMultiBodyConstraint_finalizeMultiDof)(void*);
void*(btSoftBody_Body_new)();
void(btRigidBody_clearForces)(void*);
void(btSoftBody_Anchor_setNode)(void*,void*);
void(btCollisionWorld_ConvexResultCallback_delete)(void*);
void(btSoftBody_Anchor_setLocal)(void*,const void*);
void(btSoftBody_Anchor_setInfluence)(void*,float);
void(btSoftBody_Joint_getDrift)(void*,void*);
void(btSoftBody_Anchor_setC1)(void*,const void*);
bool(btDbvtBroadphase_getReleasepaircache)(void*);
void(btSoftBody_Anchor_setC0)(void*,const void*);
void(btTranslationalLimitMotor2_testLimitValue)(void*,int,float);
int(btPersistentManifold_getCompanionIdA)(void*);
void(btSoftBody_Anchor_setBody)(void*,void*);
float(btSoftBody_Anchor_getC2)(void*);
float(btRotationalLimitMotor_getStopCFM)(void*);
void(btSoftBody_Anchor_getC1)(void*,void*);
void(btSoftBody_Anchor_getC0)(void*,void*);
void*(btNNCGConstraintSolver_new)();
void(btSoftBody_AJoint_setIcontrol)(void*,void*);
void*(btSoftBody_AJoint_getIcontrol)(void*);
int(btDbvt_getLkhd)(void*);
void*(btSoftBody_AJoint_getAxis)(void*);
bool(btGImpactBvh_isLeafNode)(void*,int);
void(btDbvt_sStkCLN_setNode)(void*,const void*);
void(btSoftBody_AJoint_Specs_setIcontrol)(void*,void*);
void(btGImpactShapeInterface_unlockChildShapes)(void*);
void(btSoftBody_AJoint_Specs_setAxis)(void*,const void*);
void(btSoftBody_addForce)(void*,const void*);
void*(btDbvt_IClone_new)();
const void*(btGImpactQuantizedBvh_get_node_pointer)(void*,int);
float(btAngularLimit_getRelaxationFactor)(void*);
void(btMultiBody_fillContactJacobianMultiDof)(void*,int,const void*,const void*,float*,void*,void*,void*);
void(btWorldImporter_deleteAllData)(void*);
void(btSoftBody_AJoint_IControl_Prepare)(void*,void*);
void(btSoftBody_AJoint_IControlWrapper_setWrapperData)(void*,void*);
void(btTranslationalLimitMotor2_getMotorCFM)(void*,void*);
void*(btManifoldPoint_new)();
void*(btSoftBody_AJoint_IControlWrapper_new)(void*,void*);
void(btHingeConstraint_setMaxMotorImpulse)(void*,float);
const char*(btWorldImporter_getNameForPointer)(void*,const void*);
void(btSoftBodyWorldInfo_setWater_offset)(void*,float);
void(btSoftBodyWorldInfo_setWater_normal)(void*,const void*);
void(btSoftBodyWorldInfo_setWater_density)(void*,float);
float(btCollisionObject_getCcdSweptSphereRadius)(void*);
void(btAABB_getMin)(void*,void*);
void(btSoftBodyWorldInfo_setMaxDisplacement)(void*,float);
void(btSoftBody_getWindVelocity)(void*,void*);
void(btSoftBodyWorldInfo_setDispatcher)(void*,void*);
void(btSoftBodyWorldInfo_setBroadphase)(void*,void*);
void*(btTriangleMeshShape_getMeshInterface)(void*);
float(btRigidBody_btRigidBodyConstructionInfo_getFriction)(void*);
void*(btWheelInfo_new)(void*);
int(btRigidBody_getFrictionSolverType)(void*);
void(btSoftBodyWorldInfo_getWater_normal)(void*,void*);
void(btMultiBodySolverConstraint_setFrictionIndex)(void*,int);
float(btSoftBodyWorldInfo_getMaxDisplacement)(void*);
void(btHingeConstraint_setUseReferenceFrameA)(void*,bool);
void(btConvexConcaveCollisionAlgorithm_clearCache)(void*);
void(btSoftBodyWorldInfo_getGravity)(void*,void*);
void*(btGeneric6DofSpring2Constraint_getTranslationalLimitMotor)(void*);
void*(btSoftBodyWorldInfo_getDispatcher)(void*);
void*(btBvhTriangleMeshShape_getTriangleInfoMap)(void*);
void*(btCompoundShape_getChildList)(void*);
int(btContactSolverInfoData_getNumIterations)(void*);
void(btBvhTriangleMeshShape_buildOptimizedBvh)(void*);
const void*(btAxisSweep3_getOverlappingPairUserCallback)(void*);
void*(btGeneric6DofSpringConstraint_new2)(void*,const void*,bool);
float(btSoftBodyWorldInfo_getAir_density)(void*);
float(btMultiBodySolverConstraint_getJacDiagABInv)(void*);
void(btManifoldPoint_setCombinedContactStiffness1)(void*,float);
void(btSliderConstraint_testLinLimits)(void*);
void*(btMultiBodySolverConstraint_getMultiBodyB)(void*);
bool(btSoftBody_getBUpdateRtCst)(void*);
void*(btEmptyShape_new)();
void(btTriangle_getVertex0)(void*,void*);
void(btSliderConstraint_setUpperAngLimit)(void*,float);
void(btSliderConstraint_setTargetLinMotorVelocity)(void*,float);
float(btHinge2Constraint_getAngle2)(void*);
void(btSliderConstraint_setTargetAngMotorVelocity)(void*,float);
void(btSoftBody_Node_getN)(void*,void*);
float(btSliderConstraint_getDampingDirLin)(void*);
void*(btDefaultVehicleRaycaster_new)(void*);
void(btSliderConstraint_setSoftnessLimAng)(void*,float);
void(btSliderConstraint_setSoftnessDirAng)(void*,float);
void(btSliderConstraint_setRestitutionOrthoLin)(void*,float);
void(btRigidBody_btRigidBodyConstructionInfo_setLinearSleepingThreshold)(void*,float);
void*(btDbvtNode_getVolume)(void*);
void(btBroadphasePair_setAlgorithm)(void*,void*);
void(btOverlappingPairCallback_removeOverlappingPairsContainingProxy)(void*,void*,void*);
void(btJointFeedback_setAppliedForceBodyB)(void*,const void*);
void(btSoftBody_appendLinearJoint4)(void*,const void*,void*,void*);
void(btTranslationalLimitMotor_delete)(void*);
void(btCollisionShape_calculateLocalInertia)(void*,float,void*);
void*(btConvexConvexAlgorithm_CreateFunc_new)(void*);
void(btSliderConstraint_setPoweredLinMotor)(void*,bool);
void(btMultiBodyDynamicsWorld_forwardKinematics)(void*);
void*(btOverlappingPairCache_findPair)(void*,void*,void*);
void(btSliderConstraint_setPoweredAngMotor)(void*,bool);
void(btSliderConstraint_setMaxLinMotorForce)(void*,float);
void(btTriangleIndexVertexMaterialArray_getLockedMaterialBase)(void*,unsigned char**,int*,int*,int*,unsigned char**,int*,int*,int*,int);
float(btSoftBody_Node_getIm)(void*);
void*(btOverlappingPairCallback_removeOverlappingPair)(void*,void*,void*,void*);
void(btSliderConstraint_setLowerAngLimit)(void*,float);
void(btDbvtBroadphase_setPrediction)(void*,float);
void(btContactSolverInfoData_setSplitImpulsePenetrationThreshold)(void*,float);
void(btStorageResult_setNormalOnSurfaceB)(void*,const void*);
void(btSliderConstraint_calculateTransforms)(void*,const void*,const void*);
bool(btSimulationIslandManager_getSplitIslands)(void*);
void(btSliderConstraint_setDampingOrthoAng)(void*,float);
void(btSliderConstraint_setDampingLimLin)(void*,float);
void(btSliderConstraint_setDampingLimAng)(void*,float);
void(btSoftBodyHelpers_DrawFaceTree)(void*,void*,int,int);
void(btSliderConstraint_setDampingDirAng)(void*,float);
bool(btSliderConstraint_getUseFrameOffset)(void*);
void(btGeneric6DofSpringConstraint_enableSpring)(void*,int,bool);
void(btMultibodyLink_getAppliedConstraintForce)(void*,void*);
void(btCompoundShapeChild_setChildShapeType)(void*,int);
void(btConvexHullShape_addPoint)(void*,const void*,bool);
float(btSliderConstraint_getUpperLinLimit)(void*);
float(btSliderConstraint_getUpperAngLimit)(void*);
float(btSliderConstraint_getTargetLinMotorVelocity)(void*);
void*(btTriangleInfoMap_new)();
void(btCollisionDispatcher_registerClosestPointsCreateFunc)(void*,int,int,void*);
void(btDbvtAabbMm_delete)(void*);
int(btManifoldPoint_getPartId1)(void*);
bool(btSliderConstraint_getSolveLinLimit)(void*);
bool(btSliderConstraint_getSolveAngLimit)(void*);
float(btSliderConstraint_getSoftnessLimLin)(void*);
float(btSliderConstraint_getTargetAngMotorVelocity)(void*);
void(btContactSolverInfoData_setRestitution)(void*,float);
int(btAlignedObjectArray_btSoftBody_Node_index_of)(void*,void*);
void*(btWorldImporter_getCollisionShapeByIndex)(void*,int);
float(btSliderConstraint_getRestitutionOrthoAng)(void*);
void(btMultibodyLink_setLinkName)(void*,const char*);
bool(btGImpactBvh_boxQuery)(void*,const void*,void*);
void(btAlignedObjectArray_btVector3_at)(void*,int,void*);
float(btMultiBodySolverConstraint_getFriction)(void*);
int(btCompoundShape_getNumChildShapes)(void*);
float(btSliderConstraint_getRestitutionDirAng)(void*);
float(btSoftBody_CJoint_getFriction)(void*);
bool(btSliderConstraint_getPoweredAngMotor)(void*);
float(btSliderConstraint_getMaxLinMotorForce)(void*);
float(btSliderConstraint_getLowerLinLimit)(void*);
float(btSliderConstraint_getLowerAngLimit)(void*);
void(btTranslationalLimitMotor_getUpperLimit)(void*,void*);
int(btBox2dShape_getVertexCount)(void*);
void(btHingeConstraint_setLimit)(void*,float,float);
float(btSliderConstraint_getLinDepth)(void*);
void(btSliderConstraint_getInfo2NonVirtual)(void*,void*,const void*,const void*,const void*,const void*,float,float);
void(btPairSet_delete)(void*);
void**(btDbvtNode_getChilds)(void*);
void(btSliderConstraint_getInfo1NonVirtual)(void*,void*);
bool(btBvhTree_isLeafNode)(void*,int);
void(btDbvtAabbMm_Mins)(void*,void*);
void(btAlignedObjectArray_btSoftBody_JointPtr_push_back)(void**,void*);
void(btDbvtBroadphase_collide)(void*,void*);
float(btWheelInfo_getEngineForce)(void*);
float(btSliderConstraint_getDampingOrthoAng)(void*);
void(btSliderConstraint_setSoftnessOrthoAng)(void*,float);
void(btManifoldPoint_getLocalPointA)(void*,void*);
float(btSliderConstraint_getDampingDirAng)(void*);
void(btSubSimplexClosestResult_delete)(void*);
float(btSliderConstraint_getAngDepth)(void*);
float(btGeneric6DofSpring2Constraint_getRelativePivotPosition)(void*,int);
void(btSliderConstraint_getAncorInA)(void*,void*);
void(btDbvtProxy_setStage)(void*,int);
float(btContactSolverInfoData_getMaxErrorReduction)(void*);
void(btSimulationIslandManager_delete)(void*);
void*(btSliderConstraint_new2)(void*,const void*,bool);
void(btSimulationIslandManager_setSplitIslands)(void*,bool);
void(btBox2dShape_getCentroid)(void*,void*);
int(btRaycastVehicle_getForwardAxis)(void*);
void(btSliderConstraint_setDampingOrthoLin)(void*,float);
void(btSimulationIslandManager_findUnions)(void*,void*,void*);
void(btCollisionObject_forceActivationState)(void*,int);
void(btMotionState_getWorldTransform)(void*,void*);
float(btRigidBody_getAngularDamping)(void*);
void(btSimulationIslandManager_buildAndProcessIslands)(void*,void*,void*,void*);
void(btPrimitiveTriangle_buildTriPlane)(void*);
void(btSimulationIslandManager_IslandCallback_delete)(void*);
void(btSimulationIslandManager_IslandCallback_processIsland)(void*,void**,int,void**,int,int);
void(btShapeHull_delete)(void*);
void(btRigidBody_saveKinematicState)(void*,float);
int(btPolyhedralConvexShape_getNumVertices)(void*);
bool*(btTranslationalLimitMotor2_getSpringStiffnessLimited)(void*);
int(btShapeHull_numIndices)(void*);
const void*(btShapeHull_getVertexPointer)(void*);
void(btSoftRigidDynamicsWorld_addSoftBody3)(void*,void*,int,int);
bool(btCollisionObject_checkCollideWithOverride)(void*,const void*);
void(btMultiBody_addBaseConstraintTorque)(void*,const void*);
void(btMotionState_delete)(void*);
void(btGeneric6DofConstraint_setAngularLowerLimit)(void*,const void*);
void(btAlignedObjectArray_btSoftBodyPtr_push_back)(void*,void*);
void*(btShapeHull_new)(const void*);
bool(btUsageBitfield_getUsedVertexA)(void*);
int(btCollisionWorld_ContactResultCallback_getCollisionFilterGroup)(void*);
void(btKinematicCharacterController_getAngularVelocity)(void*,void*);
void(btDefaultSerializer_writeHeader)(void*,unsigned char*);
void(btRaycastVehicle_updateFriction)(void*,float);
float(btSliderConstraint_getLinearPos)(void*);
void*(btDefaultSerializer_new)();
float(btSoftBody_Config_getTimescale)(void*);
void(btTranslationalLimitMotor2_setUpperLimit)(void*,const void*);
void(btChunk_setNumber)(void*,int);
void(btChunk_setDna_nr)(void*,int);
int(btHashedOverlappingPairCache_GetCount)(void*);
void*(btChunk_getOldPtr)(void*);
bool(btDbvtBroadphase_getNeedcleanup)(void*);
int(btChunk_getDna_nr)(void*);
void*(btBoxBoxDetector_new)(const void*,const void*);
int(btChunk_getChunkCode)(void*);
void(btSequentialImpulseConstraintSolver_setRandSeed)(void*,unsigned long);
unsigned long(btSequentialImpulseConstraintSolver_getRandSeed)(void*);
void(btCollisionObjectWrapper_setIndex)(void*,int);
void(btTriangleInfo_setEdgeV0V1Angle)(void*,float);
float(btCollisionObject_getRollingFriction)(void*);
void*(btGImpactQuantizedBvh_getGlobalBox)(void*);
void*(btSequentialImpulseConstraintSolver_new)();
float(btRigidBody_btRigidBodyConstructionInfo_getRollingFriction)(void*);
void(btRigidBody_updateInertiaTensor)(void*);
void(btDispatcherInfo_setEnableSatConvex)(void*,bool);
void(btRigidBody_updateDeactivation)(void*,float);
int(btCollisionWorld_LocalShapeInfo_getShapePart)(void*);
int(btIndexedMesh_getVertexStride)(void*);
void(btRigidBody_setSleepingThresholds)(void*,float,float);
float(btRotationalLimitMotor_getHiLimit)(void*);
void(btConeTwistConstraint_getFrameOffsetB)(void*,void*);
void(btRigidBody_setLinearFactor)(void*,const void*);
void*(btDbvtBroadphase_new)(void*);
void*(btDbvtNode_getData)(void*);
void(btRigidBody_setInvInertiaDiagLocal)(void*,const void*);
void(btRigidBody_setFrictionSolverType)(void*,int);
void(btRigidBody_setFlags)(void*,int);
void(btRigidBody_setContactSolverType)(void*,int);
bool(btCollisionObject_hasContactResponse)(void*);
void(btCollisionWorld_ClosestConvexResultCallback_getHitNormalWorld)(void*,void*);
void(btRigidBody_setCenterOfMassTransform)(void*,const void*);
void*(btDefaultCollisionConfiguration_getClosestPointsAlgorithmCreateFunc)(void*,int,int);
void*(btWheelInfo_RaycastInfo_new)();
int(btManifoldPoint_getContactPointFlags)(void*);
void(btMultiBody_clearForcesAndTorques)(void*);
float(btManifoldResult_calculateCombinedRestitution)(const void*,const void*);
float(btWheelInfo_getDeltaRotation)(void*);
void(btConvexPointCloudShape_setPoints2)(void*,void*,int,bool,const void*);
bool(btRigidBody_isInWorld)(void*);
float(btDbvtBroadphase_getVelocityPrediction)(void*);
void*(btConvexConcaveCollisionAlgorithm_new)(const void*,const void*,const void*,bool);
void(btRigidBody_integrateVelocities)(void*,float);
void(btRigidBody_getTotalForce)(void*,void*);
float(btPersistentManifold_getContactProcessingThreshold)(void*);
int(btMaterialProperties_getNumMaterials)(void*);
void(btRotationalLimitMotor_delete)(void*);
void(btDbvt_sStkNP_setMask)(void*,int);
void(btMultiBody_getWorldToBaseRot)(void*,void*);
void*(btAlignedObjectArray_btSoftBodyPtr_at)(void*,int);
void(btBroadphaseProxy_getAabbMin)(void*,void*);
void(btCollisionShape_getBoundingSphere)(void*,void*,float*);
float(btRigidBody_getInvMass)(void*);
void(btGImpactMeshShapePart_TrimeshPrimitiveManager_get_indices)(void*,int,unsigned int*,unsigned int*,unsigned int*);
int(btGeneric6DofSpring2Constraint_getRotationOrder)(void*);
void(btRigidBody_getInvInertiaDiagLocal)(void*,void*);
void(btPoint2PointConstraint_getPivotInA)(void*,void*);
void*(btRigidBody_getConstraintRef)(void*,int);
void(btGImpactCollisionAlgorithm_setFace1)(void*,int);
void*(btRigidBody_getBroadphaseProxy)(void*);
int(btCollisionShape_getShapeType)(void*);
bool(btMultiBodyConstraint_isUnilateral)(void*);
void(btTranslationalLimitMotor2_getStopCFM)(void*,void*);
void(btConvexConvexAlgorithm_setLowLevelOfDetail)(void*,bool);
float(btRigidBody_getAngularSleepingThreshold)(void*);
void(btTranslationalLimitMotor_setNormalCFM)(void*,const void*);
void(btRigidBody_applyTorque)(void*,const void*);
void(btRigidBody_applyImpulse)(void*,const void*,const void*);
void*(btConvexSeparatingDistanceUtil_new)(float,float);
void*(btCollisionDispatcher_getNearCallback)(void*);
void(btBroadphaseAabbCallback_delete)(void*);
void(btConvexCast_CastResult_setHitPoint)(void*,const void*);
void(btMultiBody_setBaseOmega)(void*,const void*);
void*(btMultibodyLink_getAxes)(void*);
void(btManifoldPoint_setLocalPointA)(void*,const void*);
void(btRigidBody_applyForce)(void*,const void*,const void*);
void(btRigidBody_applyDamping)(void*,float);
void(btSoftRigidDynamicsWorld_addSoftBody)(void*,void*);
void(btRigidBody_addConstraintRef)(void*,void*);
void(btSoftBody_applyForces)(void*);
void(btConvex2dConvex2dAlgorithm_CreateFunc_setPdSolver)(void*,void*);
void(btRigidBody_btRigidBodyConstructionInfo_setRollingFriction)(void*,float);
void(btConvex2dConvex2dAlgorithm_CreateFunc_setSimplexSolver)(void*,void*);
void(btRigidBody_btRigidBodyConstructionInfo_setRestitution)(void*,float);
void(btWheelInfo_setEngineForce)(void*,float);
void(btRigidBody_btRigidBodyConstructionInfo_setMass)(void*,float);
void(btDbvt_sStkNP_delete)(void*);
void(btCollisionWorld_LocalRayResult_delete)(void*);
void(btCollisionWorld_LocalShapeInfo_setTriangleIndex)(void*,int);
void(btSliderConstraint_setRestitutionOrthoAng)(void*,float);
void(btRigidBody_btRigidBodyConstructionInfo_setLinearDamping)(void*,float);
float(btRotationalLimitMotor_getAccumulatedImpulse)(void*);
unsigned int*(btBroadphaseRayCallback_getSigns)(void*);
void(btRigidBody_btRigidBodyConstructionInfo_setFriction)(void*,float);
void(btQuantizedBvh_deSerializeDouble)(void*,void*);
void(btSoftBody_clusterImpulse)(void*,const void*,const void*);
void(btRigidBody_btRigidBodyConstructionInfo_setAngularSleepingThreshold)(void*,float);
void(btGImpactQuantizedBvh_delete)(void*);
void*(btMultiBodyConstraint_getMultiBodyB)(void*);
void(btRigidBody_btRigidBodyConstructionInfo_setAngularDamping)(void*,float);
void(btMultiBodyDynamicsWorld_debugDrawMultiBodyConstraint)(void*,void*);
void*(btConvexHullShape_getUnscaledPoints)(void*);
float(btMultiBody_getJointTorque)(void*,int);
void(btRigidBody_btRigidBodyConstructionInfo_setAdditionalDampingFactor)(void*,float);
void(btRigidBody_btRigidBodyConstructionInfo_setAdditionalDamping)(void*,bool);
void(btAlignedObjectArray_btCollisionObjectPtr_push_back)(void*,void*);
bool(btSoftBody_Cluster_getContainsAnchor)(void*);
void(btRigidBody_btRigidBodyConstructionInfo_setAdditionalAngularDampingFactor)(void*,float);
void(btRigidBody_btRigidBodyConstructionInfo_getStartWorldTransform)(void*,void*);
void*(btConeShape_new)(float,float);
void(btBvhTriangleMeshShape_performConvexcast)(void*,void*,const void*,const void*,const void*,const void*);
void(btConcaveShape_processAllTriangles)(void*,void*,const void*,const void*);
void(btCompoundShapeChild_delete)(void*);
void*(btScaledBvhTriangleMeshShape_getChildShape)(void*);
float(btRigidBody_btRigidBodyConstructionInfo_getRestitution)(void*);
int(btGImpactMeshShapePart_TrimeshPrimitiveManager_getNumfaces)(void*);
void(btDbvtBroadphase_setCid)(void*,int);
void(btHingeConstraint_getInfo2NonVirtual)(void*,void*,const void*,const void*,const void*,const void*);
void(btConvexSeparatingDistanceUtil_delete)(void*);
float(btRigidBody_btRigidBodyConstructionInfo_getMass)(void*);
void(btSoftBodySolver_copyBackToSoftBodies)(void*,bool);
void(btMultiBody_setBaseMass)(void*,float);
void(btOptimizedBvhNode_setEscapeIndex)(void*,int);
float(btRigidBody_btRigidBodyConstructionInfo_getLinearDamping)(void*);
void*(btRigidBody_btRigidBodyConstructionInfo_getCollisionShape)(void*);
float(btRigidBody_btRigidBodyConstructionInfo_getAngularSleepingThreshold)(void*);
float(btHingeConstraint_getLimitRelaxationFactor)(void*);
float(btRigidBody_btRigidBodyConstructionInfo_getAngularDamping)(void*);
void(btGjkPairDetector_setFixContactNormalDirection)(void*,int);
void(btMultiBody_setUserIndex2)(void*,int);
void(btManifoldResult_setBody1Wrap)(void*,const void*);
void(btWheelInfo_setRollInfluence)(void*,float);
void(btRaycastVehicle_updateWheelTransform2)(void*,int,bool);
void(btMultiBodyDynamicsWorld_clearMultiBodyConstraintForces)(void*);
bool(btDbvtAabbMm_Contain)(void*,const void*);
void(btRaycastVehicle_updateWheelTransform)(void*,int);
void(btDbvt_clone2)(void*,void*,void*);
void(btRaycastVehicle_updateVehicle)(void*,float);
void(btRaycastVehicle_updateSuspension)(void*,float);
bool(btPrimitiveTriangle_find_triangle_collision_clip_method)(void*,void*,void*);
bool(btGImpactShapeInterface_needsRetrieveTriangles)(void*);
void(btCollisionWorld_computeOverlappingPairs)(void*);
int(btCollisionDispatcher_getDispatcherFlags)(void*);
float(btRaycastVehicle_rayCast)(void*,void*);
void*(btRotationalLimitMotor_new)();
void(btMultiBodyJointMotor_setPositionTarget2)(void*,float,float);
void*(btRaycastVehicle_getWheelInfo2)(void*);
bool(btMultiBody_hasSelfCollision)(void*);
void*(btManifoldResult_new2)(const void*,const void*);
unsigned int(btPolarDecomposition_maxIterations)(void*);
void(btConeTwistConstraint_updateRHS)(void*,float);
void*(btRaycastVehicle_getWheelInfo)(void*,int);
int(btRaycastVehicle_getUserConstraintType)(void*);
void(btConvex2dConvex2dAlgorithm_CreateFunc_setMinimumPointsPerturbationThreshold)(void*,int);
void(btTranslationalLimitMotor_setStopERP)(void*,const void*);
int(btRaycastVehicle_getRightAxis)(void*);
int(btRaycastVehicle_getNumWheels)(void*);
const void*(btBox2dShape_getVertices)(void*);
void(btRaycastVehicle_applyEngineForce)(void*,float,int);
void(btMultibodyLink_setAxisTop)(void*,int,float,float,float);
void*(btRaycastVehicle_new)(const void*,void*,void*);
void(btDbvt_setLkhd)(void*,int);
void(btRaycastVehicle_btVehicleTuning_setSuspensionStiffness)(void*,float);
void(btRaycastVehicle_btVehicleTuning_setSuspensionDamping)(void*,float);
void(btGImpactShapeInterface_setChildTransform)(void*,int,const void*);
void(btRaycastVehicle_btVehicleTuning_setSuspensionCompression)(void*,float);
void(btRaycastVehicle_btVehicleTuning_setMaxSuspensionTravelCm)(void*,float);
int(btQuantizedBvhTree_getNodeCount)(void*);
void(btRaycastVehicle_btVehicleTuning_setMaxSuspensionForce)(void*,float);
void(btRaycastVehicle_btVehicleTuning_setFrictionSlip)(void*,float);
void(btIndexedMesh_setVertexType)(void*,int);
void(btAlignedObjectArray_btPersistentManifoldPtr_push_back)(void*,void*);
float(btRaycastVehicle_btVehicleTuning_getMaxSuspensionTravelCm)(void*);
float(btRaycastVehicle_btVehicleTuning_getMaxSuspensionForce)(void*);
void(btQuantizedBvh_unQuantize)(void*,const unsigned short*,void*);
void(btQuantizedBvh_setTraversalMode)(void*,int);
void(btSoftBody_updateArea)(void*,bool);
void(btActionInterface_updateAction)(void*,void*,float);
void(btConvexCast_CastResult_setHitTransformB)(void*,const void*);
void(btManifoldPoint_setLifeTime)(void*,int);
void(btKinematicCharacterController_setUseGhostSweepTest)(void*,bool);
int(btMultibodyLink_getDofOffset)(void*);
const char*(btQuantizedBvh_serialize2)(void*,void*,void*);
void(btStorageResult_getNormalOnSurfaceB)(void*,void*);
void(btSoftBody_appendLinearJoint)(void*,const void*,void*);
void*(btCollisionWorld_new)(void*,void*,void*);
void(btQuantizedBvh_reportBoxCastOverlappingNodex)(void*,void*,const void*,const void*,const void*,const void*);
void(btCompoundShape_removeChildShapeByIndex)(void*,int);
bool(btPersistentManifold_validContactDistance)(void*,const void*);
void(btMultiBody_localFrameToWorld)(void*,int,const void*,void*);
void(btQuantizedBvh_reportAabbOverlappingNodex)(void*,void*,const void*,const void*);
bool(btBroadphaseProxy_isConvex)(int);
void(btManifoldPoint_setAppliedImpulse)(void*,float);
void(btQuantizedBvh_quantize)(void*,unsigned short*,const void*,int);
float(btGeneric6DofSpringConstraint_getDamping)(void*,int);
void*(btQuantizedBvh_getSubtreeInfoArray)(void*);
int(btPersistentManifold_getNumContacts)(void*);
void*(btDiscreteCollisionDetectorInterface_ClosestPointInput_new)();
void*(btQuantizedBvh_getQuantizedNodeArray)(void*);
unsigned int(btQuantizedBvh_getAlignmentSerializationPadding)();
void*(btQuantizedBvh_deSerializeInPlace)(void*,unsigned int,bool);
void(btQuantizedBvh_buildInternal)(void*);
void(btMultiBodyLinkCollider_setMultiBody)(void*,void*);
float(btTriangleInfoMap_getZeroAreaThreshold)(void*);
void(btNodeOverlapCallback_processNode)(void*,int,int);
void*(btVoronoiSimplexSolver_getSimplexVectorW)(void*);
void(btOptimizedBvhNode_setSubPart)(void*,int);
float(btRigidBody_btRigidBodyConstructionInfo_getLinearSleepingThreshold)(void*);
void(btOptimizedBvhNode_setAabbMinOrg)(void*,const void*);
void*(btDefaultMotionState_new3)(const void*,const void*);
void(btOptimizedBvhNode_setAabbMaxOrg)(void*,const void*);
int(btOptimizedBvhNode_getEscapeIndex)(void*);
void(btOptimizedBvhNode_getAabbMinOrg)(void*,void*);
void*(btOptimizedBvhNode_new)();
void(btQuantizedBvhNode_delete)(void*);
void(btQuantizedBvhNode_setEscapeIndexOrTriangleIndex)(void*,int);
bool(btQuantizedBvhNode_isLeafNode)(void*);
int(btQuantizedBvhNode_getTriangleIndex)(void*);
void*(btAABB_new2)(const void*,const void*,const void*);
float(btAngularLimit_getSoftness)(void*);
void(btTriangleIndexVertexMaterialArray_getLockedReadOnlyMaterialBase)(void*,const unsigned char**,int*,int*,int*,const unsigned char**,int*,int*,int*,int);
int(btQuantizedBvhNode_getPartId)(void*);
bool(btGImpactBvh_hasHierarchy)(void*);
void*(btVehicleRaycaster_btVehicleRaycasterResult_new)();
bool(btUnionFind_isRoot)(void*,int);
void*(btQuantizedBvhNode_new)();
void*(btWorldImporter_createGimpactShape)(void*,void*);
bool(btPolyhedralConvexShape_isInside)(void*,const void*,float);
void(btMultiBodySolverConstraint_setOverrideNumSolverIterations)(void*,int);
void(btMultibodyLink_getAxisBottom)(void*,int,void*);
void(btManifoldPoint_setLocalPointB)(void*,const void*);
void(btDynamicsWorld_synchronizeMotionStates)(void*);
void(btMultiBody_setJointVelMultiDof)(void*,int,float*);
void(btSoftBody_Link_getC3)(void*,void*);
float(btCollisionWorld_RayResultCallback_getClosestHitFraction)(void*);
int(btShapeHull_numVertices)(void*);
int(btPolyhedralConvexShape_getNumEdges)(void*);
void(btManifoldPoint_getLocalPointB)(void*,void*);
int(btSoftBody_getTetraVertexData)(void*,float*);
void(btPolarDecomposition_delete)(void*);
unsigned int(btPolarDecomposition_decompose)(void*,const void*,void*,void*);
float*(btMultibodyLink_getJointPos)(void*);
void(btAlignedObjectArray_btSoftBody_Face_push_back)(void*,void*);
float(btMultiBodyConstraint_getAppliedImpulse)(void*,int);
void(btPointCollector_setHasResult)(void*,bool);
void(btDiscreteDynamicsWorld_applyGravity)(void*);
void(btDbvt_clear)(void*);
void*(btBroadphasePair_new3)(void*,void*);
void(btPointCollector_setDistance)(void*,float);
int(btAlignedObjectArray_btSoftBodyPtr_size)(void*);
void*(btHingeConstraint_new)(void*,void*,const void*,const void*,const void*,const void*,bool);
void(btPointCollector_getNormalOnBInWorld)(void*,void*);
bool(btPointCollector_getHasResult)(void*);
float(btPointCollector_getDistance)(void*);
void*(btPointCollector_new)();
void(btConvexInternalShape_getLocalScalingNV)(void*,void*);
void(btPoint2PointConstraint_updateRHS)(void*,float);
void(btGeneric6DofSpring2Constraint_getAngularLowerLimit)(void*,void*);
void(btPoint2PointConstraint_setUseSolveConstraintObsolete)(void*,bool);
void(btMultiBody_setJointPosMultiDof)(void*,int,float*);
void(btBroadphaseInterface_rayTest)(void*,const void*,const void*,void*);
void(btPoint2PointConstraint_getPivotInB)(void*,void*);
int(btRigidBody_getContactSolverType)(void*);
void(btJointFeedback_getAppliedTorqueBodyA)(void*,void*);
void(btDbvt_sStkCLN_setParent)(void*,void*);
void*(btPoint2PointConstraint_new)(void*,void*,const void*,const void*);
void(btConstraintSetting_delete)(void*);
float(btRotationalLimitMotor2_getMotorCFM)(void*);
void(btConstraintSetting_setTau)(void*,float);
int(btMultiBody_calculateSerializeBufferSize)(void*);
void*(btNullPairCache_new)();
float(btTypedConstraint_getDbgDrawSize)(void*);
void(btCharacterControllerInterface_jump)(void*);
float(btMultiBodySolverConstraint_getCfm)(void*);
void(btConstraintSetting_setDamping)(void*,float);
void(btSubSimplexClosestResult_reset)(void*);
void(btGhostObject_addOverlappingObjectInternal)(void*,void*,void*);
float(btGearConstraint_getRatio)(void*);
void*(btAlignedObjectArray_btSoftBody_Anchor_at)(void*,int);
void(btMaterialProperties_setTriangleMaterialStride)(void*,int);
void(btMultiBody_computeAccelerationsArticulatedBodyAlgorithmMultiDof)(void*,float,void*,void*,void*,bool);
void(btGeneric6DofSpring2Constraint_setDamping)(void*,int,float,bool);
void(btHingeConstraint_enableAngularMotor)(void*,bool,float,float);
void(btGImpactQuantizedBvh_getNodeTriangle)(void*,int,void*);
void(btPersistentManifold_setNumContacts)(void*,int);
void(btPersistentManifold_setIndex1a)(void*,int);
void*(btBox2dBox2dCollisionAlgorithm_new2)(void*,const void*,const void*,const void*);
void(btCollisionWorld_objectQuerySingleInternal)(const void*,const void*,const void*,const void*,void*,float);
void(btPersistentManifold_setContactProcessingThreshold)(void*,float);
void(btPersistentManifold_setContactBreakingThreshold)(void*,float);
void*(btMultiSphereShape_new2)(const void*,const float*,int);
void(btTranslationalLimitMotor_getCurrentLimitError)(void*,void*);
void(btPersistentManifold_setCompanionIdB)(void*,int);
void(btPersistentManifold_setCompanionIdA)(void*,int);
void(btDynamicsWorld_setConstraintSolver)(void*,void*);
int(btGImpactMeshShapePart_TrimeshPrimitiveManager_getPart)(void*);
void(btGeneric6DofSpring2Constraint_getFrameOffsetA)(void*,void*);
void*(btBvhTree_new)();
int(btRaycastVehicle_getUserConstraintId)(void*);
void*(btConvexPolyhedron_getFaces)(void*);
int(btRigidBody_getNumConstraintRefs)(void*);
void*(btPersistentManifold_getContactPoint)(void*,int);
const void*(btPersistentManifold_getBody1)(void*);
const void*(btGImpactShapeInterface_getPrimitiveManager)(void*);
void(btSoftBody_RayFromToCaster_getRayFrom)(void*,void*);
void(btQuantizedBvhTree_build_tree)(void*,void*);
void(btOverlapCallback_delete)(void*);
void(btPersistentManifold_clearUserCache)(void*,void*);
void(btCollisionWorld_RayResultCallback_delete)(void*);
void(btPersistentManifold_clearManifold)(void*);
float(btRotationalLimitMotor2_getSpringDamping)(void*);
void*(btPersistentManifold_new)();
const void*(btBoxBoxDetector_getBox1)(void*);
void(btContactSolverInfoData_setSor)(void*,float);
void(btSoftBody_appendLink)(void*,int,int,void*,bool);
void*(btHashedOverlappingPairCache_getOverlapFilterCallback)(void*);
void(btChunk_setChunkCode)(void*,int);
float(btSliderConstraint_getRestitutionLimAng)(void*);
bool(btMultiBody_isAwake)(void*);
void(btMultiBodySolverConstraint_setJacBindex)(void*,int);
void*(btGImpactCollisionAlgorithm_CreateFunc_new)();
void(btMultiBody_finalizeMultiDof)(void*);
void(btRotationalLimitMotor2_setServoMotor)(void*,bool);
int(btDbvtAabbMm_Classify)(void*,const void*,float,int);
void(btMultiBody_setupRevolute)(void*,int,float,const void*,int,const void*,const void*,const void*,const void*,bool);
void(btOverlappingPairCache_setOverlapFilterCallback)(void*,void*);
void(btGImpactMeshShapePart_TrimeshPrimitiveManager_setMeshInterface)(void*,void*);
void(btOverlappingPairCache_processAllOverlappingPairs)(void*,void*,void*);
void(btConeTwistConstraint_getFrameOffsetA)(void*,void*);
bool(btHingeConstraint_getAngularOnly)(void*);
void(btWorldImporter_setDynamicsWorldInfo)(void*,const void*,const void*);
void(btOverlappingPairCache_cleanOverlappingPair)(void*,void*,void*);
void(btSliderConstraint_setSoftnessOrthoLin)(void*,float);
void*(btCollisionWorld_getCollisionObjectArray)(void*);
void*(btRigidBody_getMotionState)(void*);
void(btConvexCast_delete)(void*);
void(btOptimizedBvh_refit)(void*,void*,const void*,const void*);
void(btSoftBody_updateConstants)(void*);
bool(btNNCGConstraintSolver_getOnlyForNoneContact)(void*);
void*(btSoftBody_Anchor_getBody)(void*);
void(btMultiSphereShape_getSpherePosition)(void*,int,void*);
void(btCollisionAlgorithm_processCollision)(void*,const void*,const void*,const void*,void*);
void*(btSoftBody_getCfg)(void*);
float(btRotationalLimitMotor_getNormalCFM)(void*);
const void*(btMultimaterialTriangleMeshShape_getMaterialProperties)(void*,int,int);
void*(btMultimaterialTriangleMeshShape_new)(void*,bool,bool);
void(btMultiBodySliderConstraint_setPivotInB)(void*,const void*);
void(btMultiBodySliderConstraint_setPivotInA)(void*,const void*);
void(btMultiBodySliderConstraint_setJointAxis)(void*,const void*);
void*(btRotationalLimitMotor_new2)(const void*);
int(btGImpactCollisionAlgorithm_getPart1)(void*);
void(btMultiBodySliderConstraint_setFrameInB)(void*,const void*);
float*(btMultibodyLink_getJointTorque)(void*);
void(btMultiBodySliderConstraint_setFrameInA)(void*,const void*);
void(btGeneric6DofSpring2Constraint_setMaxMotorForce)(void*,int,float);
void(btMultiBodySliderConstraint_getPivotInB)(void*,void*);
void(btMultiBodySliderConstraint_getPivotInA)(void*,void*);
void(btMultiBodySliderConstraint_getJointAxis)(void*,void*);
void(btBox2dShape_getHalfExtentsWithoutMargin)(void*,void*);
void(btMultiBodySolverConstraint_delete)(void*);
void(btMultiBodySolverConstraint_setUpperLimit)(void*,float);
void(btMultiBodySolverConstraint_setUnusedPadding4)(void*,float);
void(btMultiBodySolverConstraint_setSolverBodyIdB)(void*,int);
void*(btGeneric6DofSpring2Constraint_new2)(void*,const void*,int);
void(btMultiBodySolverConstraint_setSolverBodyIdA)(void*,int);
void(btMultiBodySolverConstraint_setRhsPenetration)(void*,float);
void(btMultiBodySolverConstraint_setRhs)(void*,float);
void*(btGImpactMeshShapePart_new)();
void(btConvexShape_project)(void*,const void*,const void*,float*,float*,void*,void*);
float(btHingeConstraint_getLimitSoftness)(void*);
void(btMultiBodySolverConstraint_setRelpos2CrossNormal)(void*,const void*);
void(btMultiBodySolverConstraint_setRelpos1CrossNormal)(void*,const void*);
bool(btPolyhedralConvexShape_initializePolyhedralFeatures)(void*,int);
void(btDbvtAabbMm_tMaxs)(void*,void*);
void*(btHingeConstraint_new3)(void*,void*,const void*,const void*,bool);
void*(btAxisSweep3_getHandle)(void*,unsigned short);
void*(btAABB_new3)(const void*,const void*,const void*,float);
void(btMultiBodySolverConstraint_setOriginalContactPoint)(void*,void*);
void(btJointFeedback_setAppliedTorqueBodyB)(void*,const void*);
void(btMultiBodySolverConstraint_setOrgConstraint)(void*,void*);
void(btMultiBodySolverConstraint_setMultiBodyB)(void*,void*);
void(btHingeConstraint_setFrames)(void*,const void*,const void*);
void(btMultiBodySolverConstraint_setMultiBodyA)(void*,void*);
void*(btRigidBody_upcast)(void*);
void*(btRotationalLimitMotor2_new2)(const void*);
void(btMultiBodySolverConstraint_setLinkB)(void*,int);
int(btTypedConstraint_getOverrideNumSolverIterations)(void*);
void(btMultiBodySolverConstraint_setJacDiagABInv)(void*,float);
void*(btOverlapFilterCallbackWrapper_new)(void*);
void*(btWorldImporter_getTriangleInfoMapByIndex)(void*,int);
void*(btSoftBodyWorldInfo_getSparsesdf)(void*);
void*(btCompoundCompoundCollisionAlgorithm_CreateFunc_new)();
const void*(btQuantizedBvhTree_get_node_pointer)(void*,int);
void(btMultiBodySolverConstraint_setFriction)(void*,float);
void(btMultiBodySolverConstraint_setDeltaVelBindex)(void*,int);
void(btWheelInfo_RaycastInfo_getWheelAxleWS)(void*,void*);
void(btMultiBodySolverConstraint_setContactNormal2)(void*,const void*);
void(btMultiBodySolverConstraint_setContactNormal1)(void*,const void*);
void(btUsageBitfield_setUsedVertexA)(void*,bool);
void(btContactSolverInfoData_setSplitImpulseTurnErp)(void*,float);
void(btWheelInfo_setWheelAxleCS)(void*,const void*);
void(btMultiBodySolverConstraint_setAppliedImpulse)(void*,float);
void(btMultiBodySolverConstraint_setAngularComponentB)(void*,const void*);
void*(btManifoldResult_new)();
void(btMultiBodySolverConstraint_setAngularComponentA)(void*,const void*);
void*(btCollisionWorld_LocalRayResult_getLocalShapeInfo)(void*);
void(btAlignedObjectArray_btBroadphasePair_resizeNoInitialize)(void*,int);
float(btMultiBodySolverConstraint_getUpperLimit)(void*);
void(btGImpactCollisionAlgorithm_gimpact_vs_gimpact)(void*,const void*,const void*,const void*,const void*);
float(btMultiBodySolverConstraint_getUnusedPadding4)(void*);
void(btBroadphaseInterface_printStats)(void*);
void(btRotationalLimitMotor2_setLoLimit)(void*,float);
float(btMultiBodySolverConstraint_getRhs)(void*);
float(btWheelInfo_getMaxSuspensionForce)(void*);
void(btSliderConstraint_setRestitutionLimAng)(void*,float);
void(btConeTwistConstraint_setDamping)(void*,float);
bool(btBvhTriangleMeshShape_usesQuantizedAabbCompression)(void*);
float(btCapsuleShape_getHalfHeight)(void*);
bool(btGImpactBvh_boxQueryTrans)(void*,const void*,const void*,void*);
void(btPolyhedralConvexShape_getVertex)(void*,int,void*);
void*(btSoftBody_sRayCast_new)();
void(btMultiBodyFixedConstraint_setFrameInA)(void*,const void*);
void(btCylinderShape_getHalfExtentsWithoutMargin)(void*,void*);
void*(btSoftBody_SContact_new)();
int(btMultiBodySolverConstraint_getLinkB)(void*);
void*(btGImpactMeshShapePart_getTrimeshPrimitiveManager)(void*);
float(btCollisionWorld_ContactResultCallback_addSingleResult)(void*,void*,const void*,int,int,const void*,int,int);
int(btMultiBodySolverConstraint_getLinkA)(void*);
int(btMultiBodySolverConstraint_getJacBindex)(void*);
int(btMultiBodySolverConstraint_getFrictionIndex)(void*);
int(btQuantizedBvhNode_getEscapeIndexOrTriangleIndex)(void*);
float(btKinematicCharacterController_getMaxSlope)(void*);
void(btMultiBodySolverConstraint_getContactNormal1)(void*,void*);
void(btMultiBodySolverConstraint_getAngularComponentB)(void*,void*);
void(btConvexCast_CastResult_getHitTransformA)(void*,void*);
bool(btHingeConstraint_getUseFrameOffset)(void*);
void(btConvexTriangleCallback_setTriangleCount)(void*,int);
void(btMultiBodyPoint2Point_setPivotInB)(void*,const void*);
void(btMultiBodyPoint2Point_getPivotInB)(void*,void*);
void(btContactSolverInfoData_setMaxErrorReduction)(void*,float);
int(btCompoundShape_getUpdateRevision)(void*);
void(btCollisionWorld_ClosestConvexResultCallback_getHitPointWorld)(void*,void*);
void(btGeneric6DofConstraint_getAxis)(void*,int,void*);
void*(btQuantizedBvh_new)();
const void*(btBoxBoxDetector_getBox2)(void*);
void(btMultiBodyLinkCollider_setLink)(void*,int);
void*(btMultiBodyLinkCollider_getMultiBody)(void*);
void(btBroadphaseInterface_rayTest3)(void*,const void*,const void*,void*,const void*,const void*);
bool(btBroadphaseProxy_isConvex2d)(int);
void*(btMultiBodyLinkCollider_new)(void*,int);
bool(btAxisSweep3_testAabbOverlap)(void*,void*,void*);
void(btMultibodyLink_updateCacheMultiDof)(void*,float*);
void(btMultibodyLink_setUserPtr)(void*,const void*);
void(btMultibodyLink_setZeroRotParentToThis)(void*,const void*);
void(btMultibodyLink_setParent)(void*,int);
void(btMultibodyLink_setMass)(void*,float);
void(btDbvtNode_setDataAsInt)(void*,int);
void*(btConvexPlaneCollisionAlgorithm_CreateFunc_new)();
float(btSliderConstraint_getRestitutionLimLin)(void*);
void(btMultibodyLink_setJointType)(void*,int);
void(btMultibodyLink_setJointName)(void*,const char*);
void*(btManifoldResult_getPersistentManifold)(void*);
void(btMultibodyLink_setJointFriction)(void*,float);
float(btMultiBody_getAngularDamping)(void*);
void(btManifoldPoint_setPartId1)(void*,int);
void(btMultibodyLink_setFlags)(void*,int);
const void*(btDbvt_sStkNN_getB)(void*);
void(btSoftBody_setBUpdateRtCst)(void*,bool);
void(btMultibodyLink_setDVector)(void*,const void*);
void(btMultibodyLink_setDofOffset)(void*,int);
void(btMultibodyLink_setDofCount)(void*,int);
int(btConeShape_getConeUpIndex)(void*);
void(btMultibodyLink_setCollider)(void*,void*);
int(btSoftBody_Impulse_getAsVelocity)(void*);
void(btMultibodyLink_setCachedWorldTransform)(void*,const void*);
void(btMultibodyLink_setCachedRVector)(void*,const void*);
bool(btHingeConstraint_getEnableAngularMotor)(void*);
bool(btCollisionWorld_ContactResultCallback_needsCollision)(void*,void*);
void(btMultibodyLink_setCachedRotParentToThis)(void*,const void*);
void(btMultibodyLink_setAxisTop2)(void*,int,const void*);
void*(btRaycastVehicle_addWheel)(void*,const void*,const void*,const void*,float,float,const void*,bool);
void(btMultibodyLink_setAxisBottom)(void*,int,float,float,float);
void(btMultiBody_setWorldToBaseRot)(void*,const void*);
void(btConstraintSolver_delete)(void*);
const unsigned int*(btShapeHull_getIndexPointer)(void*);
void(btMultibodyLink_setAppliedForce)(void*,const void*);
void(btSoftBody_updatePose)(void*);
void(btMultibodyLink_setAppliedConstraintForce)(void*,const void*);
void(btMultibodyLink_getDVector)(void*,void*);
void*(btBvhTriangleMeshShape_getOptimizedBvh)(void*);
int(btCollisionWorld_ContactResultCallback_getCollisionFilterMask)(void*);
void(btConvexShape_localGetSupportingVertexWithoutMargin)(void*,const void*,void*);
void(btSoftBody_Cluster_getCom)(void*,void*);
void(btGeneric6DofSpring2Constraint_enableMotor)(void*,int,bool);
float(btCollisionObject_getSpinningFriction)(void*);
bool(btRotationalLimitMotor2_getEnableSpring)(void*);
float(btSoftBody_Config_getKDP)(void*);
bool(btConeTwistConstraint_isMotorEnabled)(void*);
void(btDefaultCollisionConstructionInfo_delete)(void*);
void(btDefaultCollisionConstructionInfo_setCollisionAlgorithmPool)(void*,void*);
void(btConeTwistConstraint_calcAngleInfo)(void*);
void*(btDefaultCollisionConstructionInfo_getPersistentManifoldPool)(void*);
int(btSoftBody_Config_getCiterations)(void*);
int(btDynamicsWorld_getWorldType)(void*);
void*(btAlignedObjectArray_btPersistentManifoldPtr_at)(void*,int);
void(btAlignedObjectArray_btSoftBody_Link_resizeNoInitialize)(void*,int);
void*(btRigidBody_btRigidBodyConstructionInfo_getMotionState)(void*);
bool(btConvexPenetrationDepthSolver_calcPenDepth)(void*,void*,const void*,const void*,const void*,const void*,void*,void*,void*,void*);
int(btDbvtBroadphase_getCupdates)(void*);
int(btDbvtProxy_getStage)(void*);
void(btRotationalLimitMotor_setDamping)(void*,float);
int(btDbvt_array_index_of)(void*,void*,int);
bool(btDbvt_update4)(void*,void*,void*,float);
void*(btFace_getIndices)(void*);
void(btMultiBody_setPosUpdated)(void*,bool);
void(btDbvt_remove)(void*,void*);
float(btKinematicCharacterController_getLinearDamping)(void*);
int(btMultiBodySolverConstraint_getOverrideNumSolverIterations)(void*);
void*(btHinge2Constraint_new)(void*,void*,void*,void*,void*);
void(btRigidBody_getLocalInertia)(void*,void*);
void(btGImpactQuantizedBvh_setPrimitiveManager)(void*,void*);
void(btConvexCast_CastResult_setAllowedPenetration)(void*,float);
const unsigned char*(btMaterialProperties_getMaterialBase)(void*);
void(btUniversalConstraint_getAxis1)(void*,void*);
int(btAlignedObjectArray_btSoftBody_Node_size)(void*);
void(btMultiBody_addLinkConstraintTorque)(void*,int,const void*);
float(btCollisionShape_getMargin)(void*);
void(btMultiBody_addBaseForce)(void*,const void*);
void*(btBroadphasePair_getAlgorithm)(void*);
int(btDbvt_allocate)(void*,void*,const void*);
float(btDbvt_sStkNPS_getValue)(void*);
void(btSoftBody_setWorldInfo)(void*,void*);
float(btCollisionShape_getContactBreakingThreshold)(void*,float);
void*(btAlignedObjectArray_btSoftBody_Face_at)(void*,int);
void(btDbvt_sStkNP_setNode)(void*,const void*);
int(btMultiBody_getNumPosVars)(void*);
float(btMultiBody_getBaseMass)(void*);
float(btRotationalLimitMotor_getStopERP)(void*);
float(btConeShape_getRadius)(void*);
void(btDynamicsWorld_addRigidBody2)(void*,void*,int,int);
void*(btBroadphaseInterface_createProxy)(void*,const void*,const void*,int,void*,int,int,void*);
void*(btGeneric6DofConstraint_getRotationalLimitMotor)(void*,int);
void(btGeneric6DofConstraint_getAngularLowerLimit)(void*,void*);
int(btDefaultCollisionConstructionInfo_getCustomCollisionAlgorithmMaxElementSize)(void*);
float*(btTypedConstraint_btConstraintInfo2_getCfm)(void*);
void*(btMultiBodyDynamicsWorld_getMultiBodyConstraint)(void*,int);
void*(btDbvt_sStkCLN_new)(const void*,void*);
float(btManifoldResult_calculateCombinedContactDamping)(const void*,const void*);
float(btRotationalLimitMotor_getBounce)(void*);
bool(btCollisionShape_isPolyhedral)(void*);
void*(btHingeConstraint_new2)(void*,const void*,const void*,bool);
void*(btCompoundCompoundCollisionAlgorithm_new)(const void*,const void*,const void*,bool);
void(btGImpactCollisionAlgorithm_gimpact_vs_compoundshape)(void*,const void*,const void*,const void*,const void*,bool);
float(btCollisionObject_getContactDamping)(void*);
void(btManifoldPoint_setCombinedRollingFriction)(void*,float);
void(btDbvtAabbMm_SignedExpand)(void*,const void*);
void*(btDbvtAabbMm_FromPoints)(const void**,int);
void*(btDefaultSoftBodySolver_new)();
void*(btCompoundCollisionAlgorithm_getChildAlgorithm)(void*,int);
float(btRotationalLimitMotor2_getEquilibriumPoint)(void*);
int(btManifoldPoint_getLifeTime)(void*);
void*(btCollisionWorld_AllHitsRayResultCallback_getHitPointWorld)(void*);
void(btGeneric6DofSpring2Constraint_setEquilibriumPoint2)(void*,int,float);
float(btWheelInfo_getClippedInvContactDotSuspension)(void*);
float(btManifoldPoint_getContactERP)(void*);
void(btDbvt_setLeaves)(void*,int);
void(btDbvtBroadphase_setFixedleft)(void*,int);
void(btDynamicsWorld_getGravity)(void*,void*);
void*(btConvex2dConvex2dAlgorithm_CreateFunc_new)(void*,void*);
void(btContactSolverInfoData_setErp)(void*,float);
void(btGImpactBvh_getNodeTriangle)(void*,int,void*);
void(btConvexShape_getAabbNonVirtual)(void*,const void*,void*,void*);
bool(btBroadphaseProxy_isSoftBody)(int);
float(btDbvtBroadphase_getPrediction)(void*);
void(btKinematicCharacterController_setFallSpeed)(void*,float);
float(btAngularLimit_getHalfRange)(void*);
float(btRotationalLimitMotor_getCurrentPosition)(void*);
void(btCollisionWorld_ClosestConvexResultCallback_setConvexToWorld)(void*,const void*);
void(btConvexConvexAlgorithm_CreateFunc_setPdSolver)(void*,void*);
int(btGImpactMeshShapePart_getPart)(void*);
void(btSliderConstraint_setFrames)(void*,const void*,const void*);
float(btContactSolverInfoData_getRestitution)(void*);
void(btRigidBody_proceedToTransform)(void*,const void*);
void(btSoftBody_initializeFaceTree)(void*);
bool(btMultiBody_isUsingGlobalVelocities)(void*);
int(btContactSolverInfoData_getSplitImpulse)(void*);
void(btCollisionDispatcher_setDispatcherFlags)(void*,int);
int(btCollisionWorld_ConvexResultCallback_getCollisionFilterMask)(void*);
void(btCollisionObject_setIslandTag)(void*,int);
void*(btConeShapeZ_new)(float,float);
void(btConeTwistConstraint_setMaxMotorImpulseNormalized)(void*,float);
const char*(btMultibodyLink_getLinkName)(void*);
void(btCollisionWorld_ClosestRayResultCallback_getRayFromWorld)(void*,void*);
void*(btDbvt_sStkNN_new)();
void(btConvexCast_CastResult_setDebugDrawer)(void*,void*);
void(btAlignedObjectArray_btSoftBody_Anchor_push_back)(void*,void*);
int(btConvexHullShape_getNumPoints)(void*);
void(btConvex2dConvex2dAlgorithm_CreateFunc_setNumPerturbationIterations)(void*,int);
float(btRotationalLimitMotor_getCurrentLimitError)(void*);
int(btDynamicsWorld_stepSimulation)(void*,float,int,float);
void*(btSortedOverlappingPairCache_getOverlapFilterCallback)(void*);
void*(btGImpactMeshShapePart_TrimeshPrimitiveManager_new3)();
void(btHingeConstraint_setAngularOnly)(void*,bool);
float(btWheelInfo_getBrake)(void*);
void*(btCylinderShapeX_new2)(float,float,float);
void*(btAlignedObjectArray_btBroadphasePair_at)(void*,int);
void(btManifoldResult_setClosestPointDistanceThreshold)(void*,float);
void(btRigidBody_getLinearVelocity)(void*,void*);
void*(btGImpactQuantizedBvh_new)();
float*(btTypedConstraint_btConstraintInfo2_getJ1linearAxis)(void*);
float(btHingeConstraint_getHingeAngle2)(void*);
void(btCollisionWorld_AllHitsRayResultCallback_getRayFromWorld)(void*,void*);
void(btHingeConstraint_getBFrame)(void*,void*);
void(btConvexCast_CastResult_getHitTransformB)(void*,void*);
void*(btConvex2dShape_new)(void*);
void*(btConvex2dConvex2dAlgorithm_new)(void*,const void*,const void*,const void*,void*,void*,int,int);
void(btIndexedMesh_setIndexType)(void*,int);
unsigned int(btDbvtBroadphase_getUpdates_call)(void*);
const void*(btManifoldResult_getBody0Internal)(void*);
void*(btSoftBody_Cluster_getLeaf)(void*);
void(btCharacterControllerInterface_reset)(void*,void*);
void*(btDbvt_sStkNN_new2)(const void*,const void*);
void(btContactSolverInfoData_setWarmstartingFactor)(void*,float);
void(btMultiBodyDynamicsWorld_integrateTransforms)(void*,float);
void(btMultibodyLink_getCachedRVector)(void*,void*);
float(btConvexInternalShape_getMarginNV)(void*);
void(btCollisionObject_setHitFraction)(void*,float);
int(btGImpactShapeInterface_getGImpactShapeType)(void*);
void(btConvexPolyhedron_getLocalCenter)(void*,void*);
int(btConvexConvexAlgorithm_CreateFunc_getNumPerturbationIterations)(void*);
void*(btDbvtProxy_getLeaf)(void*);
void(btMultiBody_calcAccelerationDeltasMultiDof)(void*,const float*,float*,void*,void*);
void(btContactSolverInfoData_setRestingContactRestitutionThreshold)(void*,int);
int(btGjkPairDetector_getFixContactNormalDirection)(void*);
void(btContactSolverInfoData_setLinearSlop)(void*,float);
void(btGImpactMeshShapePart_TrimeshPrimitiveManager_getScale)(void*,void*);
void*(btSoftBody_getFaces)(void*);
void(btVoronoiSimplexSolver_getCachedP1)(void*,void*);
float(btRigidBody_getLinearSleepingThreshold)(void*);
void*(btConvex2dConvex2dAlgorithm_CreateFunc_getSimplexSolver)(void*);
float(btContactSolverInfoData_getSplitImpulsePenetrationThreshold)(void*);
int(btGImpactQuantizedBvh_getNodeCount)(void*);
int(btContactSolverInfoData_getSolverMode)(void*);
void*(btBoxBoxCollisionAlgorithm_CreateFunc_new)();
void(btManifoldPoint_setPartId0)(void*,int);
void(btGImpactMeshShapePart_TrimeshPrimitiveManager_lock)(void*);
void(btConvexPolyhedron_setME)(void*,const void*);
void*(btMultiBodySolverConstraint_new)();
float(btContactSolverInfoData_getMaxGyroscopicForce)(void*);
void(btDiscreteCollisionDetectorInterface_Result_addContactPoint)(void*,const void*,const void*,float);
float(btContactSolverInfoData_getDamping)(void*);
int(btDefaultCollisionConstructionInfo_getDefaultMaxCollisionAlgorithmPoolSize)(void*);
float(btRotationalLimitMotor_getDamping)(void*);
void(btConeTwistConstraint_setLimit)(void*,int,float);
int(btDbvtBroadphase_getCid)(void*);
float(btConeTwistConstraint_getTwistSpan)(void*);
void*(btDbvtAabbMm_FromCR)(const void*,float);
void*(btGhostObject_upcast)(void*);
void(btBroadphaseProxy_setUniqueId)(void*,int);
void(btCollisionAlgorithmConstructionInfo_setManifold)(void*,void*);
void*(btFixedConstraint_new)(void*,void*,const void*,const void*);
void(btTriangleMeshShape_getLocalAabbMin)(void*,void*);
void(btCollisionWorld_LocalConvexResult_setHitPointLocal)(void*,const void*);
void*(btMultiBodySolverConstraint_getOrgConstraint)(void*);
void(btBroadphaseProxy_setCollisionFilterGroup)(void*,int);
void(btGImpactShapeInterface_postUpdate)(void*);
void(btCollisionObjectWrapper_setCollisionObject)(void*,const void*);
void(btUsageBitfield_setUsedVertexB)(void*,bool);
void*(btGhostObject_getOverlappingPairs)(void*);
void(btConeTwistConstraint_getInfo1NonVirtual)(void*,void*);
void(btDefaultCollisionConstructionInfo_setDefaultMaxCollisionAlgorithmPoolSize)(void*,int);
int(btGImpactShapeInterface_getNumChildShapes)(void*);
void(btGImpactShapeInterface_getChildTransform)(void*,int,void*);
void(btDefaultMotionState_getGraphicsWorldTrans)(void*,void*);
void(btDbvt_sStkNPS_setValue)(void*,float);
void*(btDiscreteDynamicsWorld_new)(void*,void*,void*,void*);
const void*(btCollisionWorld_ClosestConvexResultCallback_getHitCollisionObject)(void*);
float(btConeTwistConstraint_getSwingSpan2)(void*);
bool(btGImpactQuantizedBvh_rayQuery)(void*,const void*,const void*,void*);
void(btBroadphaseRayCallback_getRayDirectionInverse)(void*,void*);
float(btManifoldPoint_getAppliedImpulseLateral2)(void*);
void(btConvexInternalShape_setSafeMargin)(void*,float,float);
void(btDbvtBroadphase_setFupdates)(void*,int);
void*(btConvexConvexAlgorithm_new)(void*,const void*,const void*,const void*,void*,int,int);
void(btGeneric6DofConstraint_getFrameOffsetA)(void*,void*);
void(btKinematicCharacterController_setGravity)(void*,const void*);
void(btCompoundShape_removeChildShape)(void*,void*);
void*(btCompoundShapeChild_getChildShape)(void*);
void(btCylinderShape_getHalfExtentsWithMargin)(void*,void*);
void*(btCollisionObject_new)();
void(btCompoundShape_recalculateLocalAabb)(void*);
float(btTranslationalLimitMotor_getRestitution)(void*);
float(btCollisionAlgorithm_calculateTimeOfImpact)(void*,void*,void*,const void*,void*);
bool(btDispatcherInfo_getUseContinuous)(void*);
bool(btMultiBody_internalNeedsJointFeedback)(void*);
void*(btWorldImporter_createConeShapeZ)(void*,float,float);
int(btChunk_getLength)(void*);
void(btCompoundShapeChild_setChildShape)(void*,void*);
void(btCollisionObject_setBroadphaseHandle)(void*,void*);
void(btCompoundShapeChild_setChildMargin)(void*,float);
float(btMultibodyLink_getJointFriction)(void*);
void(btCollisionShape_setMargin)(void*,float);
int(btTranslationalLimitMotor_testLimitValue)(void*,int,float);
void(btDispatcher_delete)(void*);
void(btCollisionObject_delete)(void*);
void*(btDbvt_ICollide_new)();
int(btConvexPlaneCollisionAlgorithm_CreateFunc_getNumPerturbationIterations)(void*);
void(btGeneric6DofConstraint_getLinearLowerLimit)(void*,void*);
void(btDynamicsWorld_clearForces)(void*);
void*(btKinematicCharacterController_new2)(void*,void*,float,const void*);
float(btCollisionWorld_LocalConvexResult_getHitFraction)(void*);
void*(btDispatcher_getInternalManifoldPool)(void*);
void*(btPoint2PointConstraint_new2)(void*,const void*);
int(btSoftBody_Link_getBbending)(void*);
void(btCollisionWorld_convexSweepTest)(void*,const void*,const void*,const void*,void*,float);
void(btQuantizedBvh_quantizeWithClamp)(void*,unsigned short*,const void*,int);
void(btMultiBody_setUserPointer)(void*,void*);
void(btGImpactMeshShapePart_TrimeshPrimitiveManager_setNumverts)(void*,int);
void(btWheelInfoConstructionInfo_delete)(void*);
void(btBroadphaseInterface_resetPool)(void*,void*);
void(btSimulationIslandManager_initUnionFind)(void*,int);
void(btGeneric6DofSpringConstraint_setEquilibriumPoint)(void*);
void*(btDefaultMotionState_getUserPointer)(void*);
void*(btAABB_new4)(const void*);
void*(btDiscreteDynamicsWorld_getSimulationIslandManager)(void*);
void(btManifoldPoint_setContactCFM)(void*,float);
void(btDispatcherInfo_setDebugDraw)(void*,void*);
float(btCollisionObject_getCcdSquareMotionThreshold)(void*);
void*(btGImpactBvh_getPrimitiveManager)(void*);
float(btConvexShape_getMarginNonVirtual)(void*);
void*(btDbvt_getRoot)(void*);
void*(btBroadphaseAabbCallbackWrapper_new)(void*);
float(btCapsuleShape_getRadius)(void*);
void(btCollisionObject_setRollingFriction)(void*,float);
void(btGImpactQuantizedBvh_setNodeBound)(void*,int,const void*);
void(btBroadphaseProxy_delete)(void*);
void*(btAABB_new)();
bool(btGeneric6DofSpring2Constraint_matrixToEulerZXY)(const void*,void*);
void(btMultibodyLink_getZeroRotParentToThis)(void*,void*);
void(btTriangleMeshShape_recalcLocalAabb)(void*);
void(btGeneric6DofSpring2Constraint_setStiffness)(void*,int,float,bool);
void(btCollisionWorld_RayResultCallback_setClosestHitFraction)(void*,float);
float(btTypedConstraint_btConstraintInfo2_getDamping)(void*);
void(btConeTwistConstraint_getMotorTarget)(void*,void*);
void*(btTriangleIndexVertexArray_new2)(int,int*,int,int,float*,int);
void(btTypedConstraint_setUserConstraintId)(void*,int);
bool(btBroadphaseProxy_isConcave)(int);
void(btDbvt_ICollide_Process2)(void*,const void*);
void(btMultiBodyFixedConstraint_getPivotInA)(void*,void*);
int(btQuantizedBvhTree_getLeftNode)(void*,int);
float(btSoftBody_Pose_getVolume)(void*);
float(btConstraintSetting_getTau)(void*);
void(btGImpactMeshShapePart_TrimeshPrimitiveManager_setVertexbase)(void*,const unsigned char*);
void*(btBroadphasePair_getPProxy1)(void*);
void(btSoftBody_SContact_getWeights)(void*,void*);
void*(btSoftBodyHelpers_CreateFromConvexHull)(void*,const float*,int,bool);
void(btBroadphaseProxy_getAabbMax)(void*,void*);
void*(btAlignedObjectArray_btSoftBody_Link_at)(void*,int);
void(btCollisionShape_setLocalScaling)(void*,const void*);
int(btSoftBody_getLinkVertexNormalData)(void*,float*);
void(btSoftBody_addAeroForceToNode)(void*,const void*,int);
void(btBroadphaseInterface_getAabb)(void*,void*,void*,void*);
void(btCollisionWorld_ClosestConvexResultCallback_setConvexFromWorld)(void*,const void*);
void(btGImpactShapeInterface_getPrimitiveTriangle)(void*,int,void*);
void*(btMultiBodyPoint2Point_new2)(void*,int,void*,int,const void*,const void*);
void(btDispatcher_clearManifold)(void*,void*);
void*(btGImpactCollisionAlgorithm_new)(const void*,const void*,const void*);
void*(btBroadphasePair_getPProxy0)(void*);
void(btManifoldResult_refreshContactPoints)(void*);
void(btRotationalLimitMotor_setNormalCFM)(void*,float);
void(btTranslationalLimitMotor2_getCurrentLimitError)(void*,void*);
float(btSoftBodySolver_getTimeScale)(void*);
void(btDbvt_IWriter_delete)(void*);
void*(btDbvt_sStkNPS_new2)(const void*,unsigned int,float);
void(btConvexPointCloudShape_setPoints)(void*,void*,int,bool);
void(btCollisionWorld_contactTest)(void*,void*,void*);
int(btConvexConvexAlgorithm_CreateFunc_getMinimumPointsPerturbationThreshold)(void*);
int(btMultiBody_getNumDofs)(void*);
void(btContactSolverInfoData_setErp2)(void*,float);
void(btTranslationalLimitMotor_setRestitution)(void*,float);
void(btCollisionShape_setUserPointer)(void*,void*);
void(btSoftBodySolver_solveConstraints)(void*,float);
void(btCollisionWorld_objectQuerySingle)(const void*,const void*,const void*,void*,const void*,const void*,void*,float);
void(btDbvtAabbMm_tMins)(void*,void*);
float(btDiscreteCollisionDetectorInterface_ClosestPointInput_getMaximumDistanceSquared)(void*);
float(btRotationalLimitMotor2_getSpringStiffness)(void*);
int(btManifoldPoint_getIndex1)(void*);
void(btOptimizedBvh_refitPartial)(void*,void*,const void*,const void*);
void(btSoftBody_addForce2)(void*,const void*,int);
void*(btBox2dBox2dCollisionAlgorithm_new)(const void*);
void*(btWorldImporter_createTriangleInfoMap)(void*);
float(btRotationalLimitMotor2_getMotorERP)(void*);
void(btConeShape_setHeight)(void*,float);
float(btRaycastVehicle_btVehicleTuning_getSuspensionStiffness)(void*);
int(btCollisionObject_getWorldArrayIndex)(void*);
void(btSoftBody_defaultCollisionHandler)(void*,const void*);
float(btConeTwistConstraint_getDamping)(void*);
void(btBoxBoxDetector_setBox1)(void*,const void*);
void(btCollisionWorld_LocalRayResult_setHitFraction)(void*,float);
void(btQuantizedBvhTree_clearNodes)(void*);
float(btCollisionObject_getDeactivationTime)(void*);
void(btAlignedObjectArray_btSoftBody_Tetra_push_back)(void*,void*);
void(btRotationalLimitMotor2_setSpringDampingLimited)(void*,bool);
void(btTranslationalLimitMotor_setAccumulatedImpulse)(void*,const void*);
void(btDefaultCollisionConfiguration_setPlaneConvexMultipointIterations)(void*,int,int);
int(btMultiBodyConstraint_getIslandIdA)(void*);
void(btPointCollector_setPointInWorld)(void*,const void*);
void(btContactSolverInfoData_setTau)(void*,float);
void(btQuantizedBvhTree_getNodeBound)(void*,int,void*);
int(btAlignedObjectArray_btSoftBody_Link_size)(void*);
void(btCompoundShape_addChildShape)(void*,const void*,void*);
void(btSoftBody_evaluateCom)(void*,void*);
int(btConvexPlaneCollisionAlgorithm_CreateFunc_getMinimumPointsPerturbationThreshold)(void*);
void(btBoxShape_getPlaneEquation)(void*,void*,int);
void(btGImpactBvh_update)(void*);
void(btTranslationalLimitMotor2_getBounce)(void*,void*);
void(btAlignedObjectArray_btSoftBody_Node_resizeNoInitialize)(void*,int);
int(btConvex2dConvex2dAlgorithm_CreateFunc_getNumPerturbationIterations)(void*);
void(btConvexShape_getAabbSlow)(void*,const void*,void*,void*);
void*(btMultibodyLink_getAbsFrameTotVelocity)(void*);
int(btGImpactMeshShapePart_TrimeshPrimitiveManager_getLock_count)(void*);
void(btMultiBody_setMaxAppliedImpulse)(void*,float);
float(btContactSolverInfoData_getFrictionErp)(void*);
void*(btBoxBoxCollisionAlgorithm_new)(const void*);
void(btMultiBodyConstraint_delete)(void*);
int(btCollisionObjectWrapper_getIndex)(void*);
void(btSliderConstraint_setRestitutionLimLin)(void*,float);
void(btCollisionWorld_ClosestRayResultCallback_getHitPointWorld)(void*,void*);
void(btKinematicCharacterController_setMaxSlope)(void*,float);
void(btPairSet_push_pair_inv)(void*,int,int);
void(btHinge2Constraint_setLowerLimit)(void*,float);
int(btAlignedObjectArray_btBroadphasePair_size)(void*);
void(btBroadphaseRayCallback_setLambda_max)(void*,float);
void(btCollisionWorld_AllHitsRayResultCallback_setRayToWorld)(void*,const void*);
void(btDbvtBroadphase_setNeedcleanup)(void*,bool);
void(btDbvt_optimizeTopDown)(void*);
void(btDbvtBroadphase_optimize)(void*);
void(btCollisionObject_getInterpolationAngularVelocity)(void*,void*);
void(btRigidBody_applyGravity)(void*);
float(btManifoldPoint_getFrictionCFM)(void*);
void*(btTriangleCallbackWrapper_new)(void*);
void(btConvexInternalShape_setSafeMargin2)(void*,const void*,float);
void(btCollisionObjectWrapper_setParent)(void*,const void*);
void(btCollisionObject_activate)(void*,bool);
void(btGeneric6DofSpring2Constraint_setLimit)(void*,int,float,float);
void(btContactSolverInfoData_setFrictionCfm)(void*,float);
void*(btMultibodyLink_getCollider)(void*);
void(btGImpactQuantizedBvh_getNodeBound)(void*,int,void*);
float(btManifoldPoint_getDistance)(void*);
bool(btCollisionWorld_ConvexResultCallback_hasHit)(void*);
float(btConeTwistConstraint_getLimit)(void*,int);
const char*(btMultiBody_serialize)(void*,void*,void*);
void*(btMultiBodyDynamicsWorld_new)(void*,void*,void*,void*);
void(btBroadphaseInterface_setAabb)(void*,void*,const void*,const void*,void*);
float(btWheelInfo_getWheelsDampingRelaxation)(void*);
void(btSparseSdf3_Initialize)(void*,int,int);
void*(btAlignedObjectArray_btSoftBody_MaterialPtr_at)(void*,int);
float(btSoftBody_Tetra_getC2)(void*);
int(btDispatcherInfo_getDispatchFunc)(void*);
void(btDiscreteCollisionDetectorInterface_ClosestPointInput_delete)(void*);
void(btCollisionWorld_addCollisionObject)(void*,void*);
float(btRotationalLimitMotor2_getStopERP)(void*);
void*(btCompoundCompoundCollisionAlgorithm_SwappedCreateFunc_new)();
void(btCollisionWorld_LocalConvexResult_getHitNormalLocal)(void*,void*);
void*(btConvex2dConvex2dAlgorithm_CreateFunc_getPdSolver)(void*);
void*(btCollisionWorld_AllHitsRayResultCallback_new)(const void*,const void*);
void(btDbvtBroadphase_performDeferredRemoval)(void*,void*);
void(btBvhTriangleMeshShape_setTriangleInfoMap)(void*,void*);
void(btGeneric6DofSpringConstraint_setEquilibriumPoint2)(void*,int);
void(btConvexPlaneCollisionAlgorithm_collideSingleContact)(void*,const void*,const void*,const void*,const void*,void*);
void(btAlignedObjectArray_btSoftBody_MaterialPtr_push_back)(void*,void*);
void(btCollisionObject_setCustomDebugColor)(void*,const void*);
void*(btCapsuleShapeZ_new)(float,float);
void(btGImpactShapeInterface_updateBound)(void*);
void(btTranslationalLimitMotor2_getStopERP)(void*,void*);
void(btHingeConstraint_enableMotor)(void*,bool);
unsigned short*(btQuantizedBvhNode_getQuantizedAabbMax)(void*);
int(btCollisionWorld_RayResultCallback_getCollisionFilterGroup)(void*);
void*(btGjkEpaPenetrationDepthSolver_new)();
int(btCollisionObject_getActivationState)(void*);
void*(btSoftRigidCollisionAlgorithm_CreateFunc_new)();
float(btGjkPairDetector_getCachedSeparatingDistance)(void*);
void(btCollisionAlgorithmCreateFunc_setSwapped)(void*,bool);
void(btMultiBody_setupPlanar)(void*,int,float,const void*,int,const void*,const void*,const void*,bool);
bool(btCollisionWorld_RayResultCallback_needsCollision)(void*,void*);
bool(btCollisionObject_mergesSimulationIslands)(void*);
void(btPersistentManifold_removeContactPoint)(void*,int);
float(btCollisionWorld_ContactResultCallback_getClosestDistanceThreshold)(void*);
void(btConvexCast_CastResult_setNormal)(void*,const void*);
int(btConstraintSolver_getSolverType)(void*);
float(btConvexPolyhedron_getRadius)(void*);
float(btManifoldPoint_getContactCFM)(void*);
bool(btGImpactBvh_isTrimesh)(void*);
void*(btMultiBody_getUserPointer)(void*);
void(btHingeConstraint_setMotorTargetVelocity)(void*,float);
int(btDbvtBroadphase_getPid)(void*);
unsigned short(btAxisSweep3_addHandle)(void*,const void*,const void*,void*,int,int,void*);
int(btBroadphaseProxy_getCollisionFilterMask)(void*);
void(btConvexPenetrationDepthSolver_delete)(void*);
void*(btSoftBodyWorldInfo_getBroadphase)(void*);
void(btManifoldPoint_setNormalWorldOnB)(void*,const void*);
void(btTypedConstraint_solveConstraintObsolete)(void*,void*,void*,float);
void*(btTypedConstraint_getFixedBody)();
bool(btDiscreteDynamicsWorld_getApplySpeculativeContactRestitution)(void*);
void(btTranslationalLimitMotor_setMaxMotorForce)(void*,const void*);
bool(btAABB_overlapping_trans_conservative)(void*,const void*,void*);
void*(btSoftBodyRigidBodyCollisionConfiguration_new)();
void*(btBvhTriangleMeshShape_new)(void*,bool,bool);
void(btGImpactMeshShapePart_TrimeshPrimitiveManager_setStride)(void*,int);
void(btAABB_find_intersection)(void*,const void*,void*);
void(btMultiBodyConstraint_allocateJacobiansMultiDof)(void*);
void(btCollisionDispatcher_setNearCallback)(void*,void*);
void(btContactSolverInfoData_setTimeStep)(void*,float);
void*(btCompoundShape_new)(bool,int);
const void*(btCollisionObjectWrapper_getCollisionObject)(void*);
void(btTranslationalLimitMotor_getTargetVelocity)(void*,void*);
void(btConeTwistConstraint_getBFrame)(void*,void*);
void*(btFace_new)();
void(btTranslationalLimitMotor_getMaxMotorForce)(void*,void*);
void(btSoftBody_RayFromToCaster_getRayNormalizedDirection)(void*,void*);
void(btContactSolverInfoData_setSplitImpulse)(void*,int);
void(btBvhTriangleMeshShape_setOptimizedBvh2)(void*,void*,const void*);
void(btCollisionObject_getInterpolationWorldTransform)(void*,void*);
void*(btIDebugDrawWrapper_getGCHandle)(void*);
void(btBroadphaseProxy_setCollisionFilterMask)(void*,int);
bool(btCollisionObject_getCustomDebugColor)(void*,void*);
void(btRotationalLimitMotor_setMaxLimitForce)(void*,float);
void(btGearConstraint_setAxisB)(void*,void*);
void(btRotationalLimitMotor_setCurrentLimit)(void*,int);
bool(btGeneric6DofSpring2Constraint_matrixToEulerYZX)(const void*,void*);
int(btCollisionObject_getIslandTag)(void*);
int(btGeneric6DofConstraint_getFlags)(void*);
float(btCollisionObject_getRestitution)(void*);
bool(btCollisionAlgorithmCreateFunc_getSwapped)(void*);
bool(btGeneric6DofConstraint_getUseSolveConstraintObsolete)(void*);
int(btCollisionObject_getUserIndex2)(void*);
void(btVoronoiSimplexSolver_getCachedP2)(void*,void*);
void(btRigidBody_setNewBroadphaseProxy)(void*,void*);
void(btTypedConstraint_enableFeedback)(void*,bool);
void*(btMultiSphereShape_new)(const float*,const float*,int);
void(btCollisionWorld_serialize)(void*,void*);
void*(btAABB_new5)(const void*,float);
void*(btCollisionObject_getUserPointer)(void*);
void(btGImpactQuantizedBvh_buildSet)(void*);
float(btRotationalLimitMotor_getTargetVelocity)(void*);
int(btGImpactMeshShapePart_TrimeshPrimitiveManager_getStride)(void*);
int(btCollisionObjectWrapper_getPartId)(void*);
void(btCompoundShape_createAabbTreeFromChildren)(void*);
bool(btOverlapFilterCallback_needBroadphaseCollision)(void*,void*,void*);
float(btConstraintSetting_getDamping)(void*);
void(btTranslationalLimitMotor2_setMotorCFM)(void*,const void*);
void(btFace_delete)(void*);
float(btRotationalLimitMotor2_getCurrentLimitError)(void*);
bool(btBroadphaseAabbCallback_process)(void*,const void*);
void*(btKinematicCharacterController_getGhostObject)(void*);
void(btCollisionWorld_ClosestConvexResultCallback_setHitNormalWorld)(void*,const void*);
float(btCollisionObject_getFriction)(void*);
const void*(btDbvt_sStkCLN_getNode)(void*);
float(btHingeConstraint_getLimitSign)(void*);
void(btRaycastVehicle_getWheelTransformWS)(void*,int,void*);
void(btUsageBitfield_reset)(void*);
int(btMultiBodyLinkCollider_getLink)(void*);
bool(btCollisionWorld_RayResultCallback_hasHit)(void*);
void(btCollisionShape_setUserIndex)(void*,int);
void*(btAlignedObjectArray_btSoftBody_ClusterPtr_at)(void*,int);
void(btConvexPolyhedron_getMC)(void*,void*);
bool(btGeneric6DofSpring2Constraint_matrixToEulerXYZ)(const void*,void*);
void(btDispatcherInfo_setUseEpa)(void*,bool);
void(btConvexShape_localGetSupportVertexNonVirtual)(void*,const void*,void*);
int(btDefaultCollisionConstructionInfo_getUseEpaPenetrationAlgorithm)(void*);
void(btBoxBoxDetector_setBox2)(void*,const void*);
void(btGImpactMeshShapePart_TrimeshPrimitiveManager_unlock)(void*);
void(btConvexPolyhedron_setRadius)(void*,float);
void*(btWheelInfo_getClientInfo)(void*);
void(btDbvt_benchmark)();
void(btConvexPolyhedron_delete)(void*);
float(btHinge2Constraint_getAngle1)(void*);
float(btCompoundShapeChild_getChildMargin)(void*);
bool(btSortedOverlappingPairCache_needsBroadphaseCollision)(void*,void*,void*);
void*(btCylinderShapeZ_new2)(float,float,float);
void(btCollisionWorld_ClosestConvexResultCallback_setHitPointWorld)(void*,const void*);
void(btCollisionWorld_ClosestRayResultCallback_getHitNormalWorld)(void*,void*);
void(btGImpactShapeInterface_getBulletTetrahedron)(void*,int,void*);
void(btActionInterface_debugDraw)(void*,void*);
void(btGeneric6DofSpring2Constraint_setAngularLowerLimit)(void*,const void*);
void(btCollisionWorld_LocalRayResult_setHitNormalLocal)(void*,const void*);
void(btDbvt_ICollide_Process3)(void*,const void*,float);
void(btCollisionObject_setUserIndex2)(void*,int);
void(btAlignedObjectArray_btPersistentManifoldPtr_resizeNoInitialize)(void*,int);
void(btCollisionWorld_addCollisionObject3)(void*,void*,int,int);
int(btCollisionWorld_LocalShapeInfo_getTriangleIndex)(void*);
float(btStorageResult_getDistance)(void*);
void(btGImpactMeshShapePart_TrimeshPrimitiveManager_setType)(void*,int);
void(btBoxShape_getHalfExtentsWithMargin)(void*,void*);
void*(btConvexPlaneCollisionAlgorithm_new)(void*,const void*,const void*,const void*,bool,int,int);
void(btDiscreteCollisionDetectorInterface_Result_setShapeIdentifiersA)(void*,int,int);
void(btDiscreteDynamicsWorld_setSynchronizeAllMotionStates)(void*,bool);
void(btCollisionWorld_ClosestRayResultCallback_setHitPointWorld)(void*,const void*);
void(btCompoundShapeChild_setTransform)(void*,const void*);
void(btSoftBody_Cluster_getAv)(void*,void*);
void*(btDefaultCollisionConstructionInfo_getCollisionAlgorithmPool)(void*);
void(btJointFeedback_getAppliedForceBodyB)(void*,void*);
void(btTranslationalLimitMotor2_setCurrentLinearDiff)(void*,const void*);
void(btCollisionAlgorithm_delete)(void*);
void(btCollisionWorld_LocalRayResult_setCollisionObject)(void*,const void*);
void(btDbvtBroadphase_setDupdates)(void*,int);
void*(btDbvtAabbMm_FromMM)(const void*,const void*);
void*(btWorldImporter_createPoint2PointConstraint)(void*,void*,const void*);
void(btCollisionWorld_ContactResultCallback_setCollisionFilterMask)(void*,int);
void*(btBox2dShape_new)(const void*);
void(btDiscreteCollisionDetectorInterface_ClosestPointInput_setTransformB)(void*,const void*);
bool(btConeTwistConstraint_getAngularOnly)(void*);
void(btCollisionObject_setRestitution)(void*,float);
void(btVehicleRaycaster_btVehicleRaycasterResult_getHitNormalInWorld)(void*,void*);
void*(btCollisionAlgorithmConstructionInfo_new2)(void*,int);
void*(btHingeAccumulatedAngleConstraint_new4)(void*,const void*,bool);
void(btContactSolverInfoData_setNumIterations)(void*,int);
int(btBvhTree_getLeftNode)(void*,int);
bool(btGeneric6DofConstraint_getUseLinearReferenceFrameA)(void*);
void(btStorageResult_setDistance)(void*,float);
void(btGeneric6DofSpring2Constraint_getCalculatedTransformB)(void*,void*);
void(btCollisionWorld_ContactResultCallback_delete)(void*);
void(btConvexTriangleMeshShape_calculatePrincipalAxisTransform)(void*,void*,void*,float*);
void(btDbvtBroadphase_setDeferedcollide)(void*,bool);
void(btDiscreteDynamicsWorld_setLatencyMotionStateInterpolation)(void*,bool);
bool(btGeneric6DofConstraint_isLimited)(void*,int);
float(btDispatcherInfo_getConvexConservativeDistanceThreshold)(void*);
bool(btDispatcherInfo_getUseConvexConservativeDistanceUtil)(void*);
void(btDispatcherInfo_setConvexConservativeDistanceThreshold)(void*,float);
int(btGImpactBvh_getRightNode)(void*,int);
void(btCollisionWorld_ConvexResultCallback_setCollisionFilterGroup)(void*,int);
void(btDispatcherInfo_setEnableSPU)(void*,bool);
void(btMultiBody_setBaseName)(void*,const char*);
void(btGeneric6DofSpring2Constraint_getFrameOffsetB)(void*,void*);
void(btCollisionWorld_LocalShapeInfo_delete)(void*);
void(btDispatcherInfo_setUseConvexConservativeDistanceUtil)(void*,bool);
void(btTriangleMesh_setWeldingThreshold)(void*,float);
int(btDbvtNode_getDataAsInt)(void*);
void(btConvexConvexAlgorithm_CreateFunc_setNumPerturbationIterations)(void*,int);
void*(btMLCPSolver_new)(void*);
void*(btSoftBody_AJoint_IControlWrapper_getWrapperData)(void*);
void(btBvhTriangleMeshShape_partialRefitTree)(void*,const void*,const void*);
bool(btDbvt_update6)(void*,void*,void*,const void*,float);
bool(btDispatcher_needsResponse)(void*,const void*,const void*);
void(btDynamicsWorld_addRigidBody)(void*,void*);
void(btSoftBody_Impulse_getVelocity)(void*,void*);
void*(btDynamicsWorld_getConstraint)(void*,int);
void*(btDynamicsWorld_getConstraintSolver)(void*);
int(btDynamicsWorld_getNumConstraints)(void*);
void(btMultiBodyJointMotor_setVelocityTarget2)(void*,float,float);
void(btDynamicsWorld_removeRigidBody)(void*,void*);
const void*(btCollisionObjectWrapper_getCollisionShape)(void*);
int(btConeTwistConstraint_getFlags)(void*);
void(btDbvt_optimizeTopDown2)(void*,int);
void(btSliderConstraint_getFrameOffsetB)(void*,void*);
void(btCollisionWorld_performDiscreteCollisionDetection)(void*);
void(btRotationalLimitMotor2_setCurrentLimitError)(void*,float);
void(btMultiBody_getBaseForce)(void*,void*);
void(btGearConstraint_getAxisB)(void*,void*);
int(btCollisionShape_calculateSerializeBufferSize)(void*);
void(btMultiBodyConstraint_createConstraintRows)(void*,void*,void*,const void*);
void(btConvexShape_localGetSupportingVertex)(void*,const void*,void*);
void(btGearConstraint_setRatio)(void*,float);
void(btGImpactMeshShapePart_TrimeshPrimitiveManager_get_bullet_triangle)(void*,int,void*);
int(btDbvtBroadphase_getDupdates)(void*);
void(btKinematicCharacterController_setAngularDamping)(void*,float);
int(btRotationalLimitMotor_getCurrentLimit)(void*);
float(btSphereShape_getRadius)(void*);
void(btRotationalLimitMotor_setTargetVelocity)(void*,float);
float(btRotationalLimitMotor2_getMaxMotorForce)(void*);
const void*(btMinkowskiSumShape_getShapeB)(void*);
int(btBroadphaseProxy_getUid)(void*);
float(btRotationalLimitMotor_getLimitSoftness)(void*);
float(btConeTwistConstraint_getMaxMotorImpulse)(void*);
float(btRotationalLimitMotor_getLoLimit)(void*);
bool(btRotationalLimitMotor_needApplyTorques)(void*);
float(btRotationalLimitMotor_solveAngularLimits)(void*,float,void*,float,void*,void*);
void(btCollisionObject_setInterpolationLinearVelocity)(void*,const void*);
int(btDbvtBroadphase_getFupdates)(void*);
void(btRotationalLimitMotor_setHiLimit)(void*,float);
void*(btGImpactCompoundShape_CompoundPrimitiveManager_getCompoundShape)(void*);
void(btMultibodyLink_setInertiaLocal)(void*,const void*);
float(btCollisionWorld_RayResultCallback_addSingleResult)(void*,void*,bool);
void(btRotationalLimitMotor_setEnableMotor)(void*,bool);
float(btConvexCast_CastResult_getFraction)(void*);
void(btRotationalLimitMotor_setLoLimit)(void*,float);
void*(btGImpactQuantizedBvh_new2)(void*);
void(btRotationalLimitMotor_setMaxMotorForce)(void*,float);
void(btRotationalLimitMotor_setStopCFM)(void*,float);
float(btContactSolverInfoData_getErp)(void*);
void(btCollisionWorld_LocalRayResult_setLocalShapeInfo)(void*,void*);
void(btStorageResult_setClosestPointInB)(void*,const void*);
void(btStorageResult_getClosestPointInB)(void*,void*);
int*(btTranslationalLimitMotor_getCurrentLimit)(void*);
int(btMultiBodyDynamicsWorld_getNumMultibodies)(void*);
void(btDbvt_write)(void*,void*);
float(btWheelInfoConstructionInfo_getWheelsDampingCompression)(void*);
void(btTranslationalLimitMotor_getNormalCFM)(void*,void*);
void*(btGImpactCompoundShape_CompoundPrimitiveManager_new2)(void*);
bool(btGImpactBvh_rayQuery)(void*,const void*,const void*,void*);
float(btHingeConstraint_getMaxMotorImpulse)(void*);
void(btTranslationalLimitMotor_setCurrentLimitError)(void*,const void*);
void(btTranslationalLimitMotor_setCurrentLinearDiff)(void*,const void*);
void(btTriangleBuffer_clearBuffer)(void*);
void(btTranslationalLimitMotor_setLimitSoftness)(void*,float);
void(btTranslationalLimitMotor_setLowerLimit)(void*,const void*);
void*(btMinkowskiSumShape_new)(const void*,const void*);
bool(btCollisionWorld_getForceUpdateAllAabbs)(void*);
int(btMultiBodySolverConstraint_getDeltaVelAindex)(void*);
void(btTranslationalLimitMotor_setStopCFM)(void*,const void*);
void(btPersistentManifold_refreshContactPoints)(void*,const void*,const void*);
float(btDbvtAabbMm_ProjectMinimum)(void*,const void*,unsigned int);
void*(btHingeConstraint_new4)(void*,const void*,bool);
float(btMultiBody_getMaxAppliedImpulse)(void*);
bool(btMultiBody_isUsingRK4Integration)(void*);
void*(btMultibodyLink_getAbsFrameLocVelocity)(void*);
void(btCollisionWorld_LocalConvexResult_getHitPointLocal)(void*,void*);
float(btTypedConstraint_internalGetAppliedImpulse)(void*);
void*(btSoftBodyRigidBodyCollisionConfiguration_new2)(const void*);
void(btGeneric6DofSpring2Constraint_setBounce)(void*,int,float);
void(btGeneric6DofSpringConstraint_setEquilibriumPoint3)(void*,int,float);
void*(btCompoundShape_getChildShape)(void*,int);
void(btWheelInfo_setWheelsRadius)(void*,float);
void*(btMultiBodyLinkCollider_upcast)(void*);
void*(btGImpactMeshShapePart_TrimeshPrimitiveManager_new2)(const void*);
void*(btPairCachingGhostObject_getOverlappingPairCache)(void*);
void(btTriangleMesh_addTriangle)(void*,const void*,const void*,const void*,bool);
bool(btSubSimplexClosestResult_getDegenerate)(void*);
void*(btDantzigSolver_new)();
void(btGeneric6DofConstraint_getLinearUpperLimit)(void*,void*);
bool(btGeneric6DofConstraint_getUseFrameOffset)(void*);
void*(btWorldImporter_createConeTwistConstraint)(void*,void*,void*,const void*,const void*);
void*(btCollisionWorld_getBroadphase)(void*);
void(btCollisionWorld_ClosestRayResultCallback_getRayToWorld)(void*,void*);
void(btGImpactBvh_find_collision)(void*,const void*,void*,const void*,void*);
void(btGeneric6DofConstraint_setLinearLowerLimit)(void*,const void*);
float(btUniversalConstraint_getAngle2)(void*);
bool(btRotationalLimitMotor2_getEnableMotor)(void*);
bool(btVoronoiSimplexSolver_closest)(void*,void*);
void(btTranslationalLimitMotor2_getCurrentLimitErrorHi)(void*,void*);
int(btPersistentManifold_addManifoldPoint)(void*,const void*,bool);
void*(btAlignedObjectArray_btSoftBody_Node_at)(void*,int);
void(btCollisionDispatcher_setCollisionConfiguration)(void*,void*);
float(btContactSolverInfoData_getGlobalCfm)(void*);
void(btRotationalLimitMotor2_setBounce)(void*,float);
void(btRotationalLimitMotor2_setCurrentLimit)(void*,int);
void(btRotationalLimitMotor2_setCurrentLimitErrorHi)(void*,float);
const unsigned char*(btGImpactMeshShapePart_TrimeshPrimitiveManager_getVertexbase)(void*);
void(btRotationalLimitMotor2_setEnableMotor)(void*,bool);
void*(btMultiBodyDynamicsWorld_getMultiBody)(void*,int);
void(btRotationalLimitMotor2_setHiLimit)(void*,float);
int(btMultiBodySolverConstraint_getSolverBodyIdB)(void*);
void(btRotationalLimitMotor2_setMotorCFM)(void*,float);
void(btRotationalLimitMotor2_setServoTarget)(void*,float);
void(btMultiBody_worldDirToLocal)(void*,int,const void*,void*);
bool(btCollisionWorld_ContactResultCallbackWrapper_needsCollision)(void*,void*);
void(btRotationalLimitMotor2_setTargetVelocity)(void*,float);
void(btBroadphaseRayCallback_setRayDirectionInverse)(void*,const void*);
void(btDefaultSoftBodySolver_copySoftBodyToVertexBuffer)(void*,const void*,void*);
float(btGeneric6DofSpringConstraint_getEquilibriumPoint)(void*,int);
void(btMultiBody_setLinearDamping)(void*,float);
void(btTranslationalLimitMotor2_getCurrentLinearDiff)(void*,void*);
float(btConeTwistConstraint_getLimitSoftness)(void*);
bool(btGImpactQuantizedBvh_isTrimesh)(void*);
void(btSoftBody_clusterVelocity)(const void*,const void*,void*);
void(btMultiBody_setAngularDamping)(void*,float);
void**(btDbvtBroadphase_getStageRoots)(void*);
void*(btCollisionWorld_ConvexResultCallbackWrapper_new)(void*,void*);
void(btSoftBody_Body_angularVelocity)(void*,const void*,void*);
void(btHinge2Constraint_setUpperLimit)(void*,float);
void*(btWorldImporter_createConeShapeX)(void*,float,float);
int(btShapeHull_numTriangles)(void*);
void(btTranslationalLimitMotor2_getUpperLimit)(void*,void*);
int(btVoronoiSimplexSolver_getNumVertices)(void*);
void(btGImpactMeshShapePart_TrimeshPrimitiveManager_setLock_count)(void*,int);
void(btDbvtBroadphase_setNewpairs)(void*,int);
void(btPersistentManifold_setBodies)(void*,const void*,const void*);
void(btKinematicCharacterController_getGravity)(void*,void*);
void(btTranslationalLimitMotor2_setLowerLimit)(void*,const void*);
void(btTranslationalLimitMotor2_setMaxMotorForce)(void*,const void*);
void(btTranslationalLimitMotor2_setMotorERP)(void*,const void*);
void(btTranslationalLimitMotor2_setSpringDamping)(void*,const void*);
void(btTranslationalLimitMotor2_setSpringStiffness)(void*,const void*);
void(btTranslationalLimitMotor2_setStopCFM)(void*,const void*);
void(btTranslationalLimitMotor2_setTargetVelocity)(void*,const void*);
void(btDefaultCollisionConstructionInfo_setDefaultMaxPersistentManifoldPoolSize)(void*,int);
void*(btSerializerWrapper_new)(void*,void*,void*,void*,void*,void*,void*,void*,void*,void*,void*,void*,void*,void*,void*);
void(btMultiBody_forwardKinematics)(void*,void*,void*);
void(btCollisionWorld_debugDrawWorld)(void*);
bool(btCollisionObject_hasAnisotropicFriction)(void*,int);
float(btGeneric6DofSpring2Constraint_getAngle)(void*,int);
void(btGeneric6DofSpring2Constraint_getAngularLowerLimitReversed)(void*,void*);
void(btGeneric6DofSpring2Constraint_getAngularUpperLimit)(void*,void*);
void(btGeneric6DofSpring2Constraint_getAngularUpperLimitReversed)(void*,void*);
bool(btBulletXmlWorldImporter_loadFile)(void*,const char*);
void(btManifoldPoint_setDistance1)(void*,float);
void(btMultiBodyConstraint_setMaxAppliedImpulse)(void*,float);
void*(btSparseSdf_new)();
void(btGeneric6DofSpring2Constraint_setEquilibriumPoint)(void*);
void(btGeneric6DofSpring2Constraint_getLinearUpperLimit)(void*,void*);
int(btManifoldPoint_getPartId0)(void*);
void(btCollisionObject_setCcdMotionThreshold)(void*,float);
void(btSoftBody_addAeroForceToFace)(void*,const void*,int);
void(btGeneric6DofSpring2Constraint_setAngularUpperLimit)(void*,const void*);
void(btGeneric6DofSpring2Constraint_setAngularUpperLimitReversed)(void*,const void*);
int(btCylinderShape_getUpAxis)(void*);
void(btRotationalLimitMotor2_setSpringStiffness)(void*,float);
void(btGeneric6DofSpring2Constraint_setEquilibriumPoint3)(void*,int);
const void*(btBvhTree_get_node_pointer2)(void*,int);
void(btGeneric6DofSpring2Constraint_setLinearLowerLimit)(void*,const void*);
void*(btTriangle_new)();
void(btGImpactQuantizedBvh_find_collision)(const void*,const void*,const void*,const void*,void*);
float(btCollisionObject_getCcdMotionThreshold)(void*);
void*(btStaticPlaneShape_new)(const void*,float);
float(btManifoldPoint_getCombinedContactStiffness1)(void*);
void*(btSoftBody_Cluster_getFramerefs)(void*);
void(btCharacterControllerInterface_jump2)(void*,const void*);
void*(btGImpactCompoundShape_CompoundPrimitiveManager_new)(const void*);
void(btGjkPairDetector_setCatchDegeneracies)(void*,int);
void(btSoftBody_Node_setX)(void*,const void*);
void(btSoftBody_appendNode)(void*,const void*,float);
void(btCollisionAlgorithm_getAllContactManifolds)(void*,void*);
void(btTriangleInfoMap_setEqualVertexThreshold)(void*,float);
void(btGeneric6DofSpring2Constraint_getCalculatedTransformA)(void*,void*);
void*(btPairCachingGhostObject_new)();
int(btSoftBody_Note_getRank)(void*);
void*(btTypedConstraint_getRigidBodyA)(void*);
void*(btCollisionObject_internalGetExtensionPointer)(void*);
void(btPrimitiveTriangle_applyTransform)(void*,const void*);
void(btChunk_setLength)(void*,int);
void(btDbvtAabbMm_Center)(void*,void*);
void(btRigidBody_computeGyroscopicForceExplicit)(void*,float,void*);
void(btGeneric6DofSpringConstraint_setStiffness)(void*,int,float);
void(btWheelInfo_setSkidInfo)(void*,float);
void(btCollisionWorld_rayTestSingleInternal)(const void*,const void*,const void*,void*);
void*(btCollisionConfiguration_getPersistentManifoldPool)(void*);
float(btMultiBodyConstraint_getPosition)(void*,int);
void*(btGImpactShapeInterface_getChildShape)(void*,int);
void*(btDefaultSerializer_new2)(int);
void(btMultiBody_localDirToWorld)(void*,int,const void*,void*);
int(btBvhTree_getNodeData)(void*,int);
void*(btWorldImporter_createGeneric6DofConstraint2)(void*,void*,const void*,bool);
bool(btGImpactShapeInterface_childrenHasTransform)(void*);
void(btManifoldPoint_setContactERP)(void*,float);
void(btPrimitiveManagerBase_delete)(void*);
void(btCollisionWorld_ConvexResultCallback_setClosestHitFraction)(void*,float);
void*(btMultiBodyFixedConstraint_new2)(void*,int,void*,int,const void*,const void*,const void*,const void*);
bool(btGeneric6DofConstraint_testAngularLimitMotor)(void*,int);
bool(btRotationalLimitMotor2_getSpringDampingLimited)(void*);
float(btDbvtBroadphase_getUpdates_ratio)(void*);
void(btTransformUtil_calculateVelocity)(const void*,const void*,float,void*,void*);
void(btGImpactBvh_getNodeBound)(void*,int,void*);
void*(btGeneric6DofConstraint_new)(void*,void*,const void*,const void*,bool);
void*(btCapsuleShape_new)(float,float);
float(btGImpactMeshShapePart_TrimeshPrimitiveManager_getMargin)(void*);
void*(btGImpactQuantizedBvh_getPrimitiveManager)(void*);
void(btDefaultMotionState_setStartWorldTrans)(void*,const void*);
void*(btHingeAccumulatedAngleConstraint_new3)(void*,void*,const void*,const void*,bool);
void*(btConvex2dShape_getChildShape)(void*);
void(btCollisionObject_setCollisionFlags)(void*,int);
void*(btSoftBody_RContact_new)();
void(btDiscreteDynamicsWorld_updateVehicles)(void*,float);
bool(btSoftBody_checkContact)(void*,const void*,const void*,float,void*);
void(btTriangleMeshShape_localGetSupportingVertex)(void*,const void*,void*);
void(btGImpactCollisionAlgorithm_setPart0)(void*,int);
void(btGjkPairDetector_setLastUsedMethod)(void*,int);
void(btSoftBody_Link_delete)(void*);
void(btAABB_get_center_extend)(void*,void*,void*);
void(btContactSolverInfoData_setSingleAxisRollingFrictionThreshold)(void*,float);
void(btDbvt_rayTestInternal)(void*,const void*,const void*,const void*,const void*,unsigned int*,float,const void*,const void*,const void*);
void(btTranslationalLimitMotor2_getServoTarget)(void*,void*);
int(btQuantizedBvhTree_getEscapeNodeIndex)(void*,int);
void(btRotationalLimitMotor2_setEquilibriumPoint)(void*,float);
void(btAngularLimit_set)(void*,float,float,float,float,float);
int(btQuantizedBvhTree_getNodeData)(void*,int);
void(btTranslationalLimitMotor_setTargetVelocity)(void*,const void*);
void(btQuantizedBvhTree_setNodeBound)(void*,int,const void*);
bool(btGImpactQuantizedBvh_boxQuery)(void*,const void*,void*);
void(btSimulationIslandManager_updateActivationState)(void*,void*,void*);
void(btRotationalLimitMotor2_setStopERP)(void*,float);
void(btConvexCast_CastResult_setHitTransformA)(void*,const void*);
void(btGjkPairDetector_setMinkowskiA)(void*,const void*);
void(btDbvtBroadphase_setCupdates)(void*,int);
void(btCollisionWorld_LocalShapeInfo_setShapePart)(void*,int);
float(btMultiBodyConstraintSolver_solveGroupCacheFriendlyFinish)(void*,void**,int,const void*);
void*(btGeneric6DofSpringConstraint_new)(void*,void*,const void*,const void*,bool);
float(btManifoldResult_calculateCombinedRollingFriction)(const void*,const void*);
void(btGImpactQuantizedBvh_update)(void*);
void(btDynamicsWorld_removeConstraint)(void*,void*);
bool(btConvexCast_calcTimeOfImpact)(void*,const void*,const void*,const void*,const void*,void*);
void(btGeneric6DofSpring2Constraint_setTargetVelocity)(void*,int,float);
void*(btConvexConcaveCollisionAlgorithm_SwappedCreateFunc_new)();
void(btGeneric6DofSpring2Constraint_setRotationOrder)(void*,int);
void*(btGImpactBvh_new)();
float(btRotationalLimitMotor2_getStopCFM)(void*);
float(btMultibodyLink_getJointDamping)(void*);
void(btRaycastVehicle_resetSuspension)(void*);
bool(btCharacterControllerInterface_onGround)(void*);
void(btGImpactShapeInterface_processAllTrianglesRay)(void*,void*,const void*,const void*);
void(btDbvt_setFree)(void*,void*);
void*(btSoftBody_new2)(void*);
void*(btCylinderShapeZ_new)(const void*);
unsigned int(btCollisionWorld_RayResultCallback_getFlags)(void*);
float(btSoftBody_SContact_getMargin)(void*);
void(btGImpactCompoundShape_CompoundPrimitiveManager_setCompoundShape)(void*,void*);
void*(btGImpactMeshShapePart_TrimeshPrimitiveManager_new)(void*,int);
int(btAlignedObjectArray_btSoftBody_Anchor_size)(void*);
int(btGImpactMeshShapePart_TrimeshPrimitiveManager_getIndexstride)(void*);
void(btAxisSweep3_removeHandle)(void*,unsigned short,void*);
void(btConvexCast_CastResult_reportFailure)(void*,int,int);
void(btSoftBody_appendTetra2)(void*,int,int,int,int,void*);
int(btGImpactMeshShapePart_TrimeshPrimitiveManager_getNumverts)(void*);
void(btMultiBody_addJointTorque)(void*,int,float);
void(btHinge2Constraint_getAnchor2)(void*,void*);
void(btGImpactMeshShapePart_TrimeshPrimitiveManager_setIndexbase)(void*,const unsigned char*);
void(btGImpactMeshShapePart_TrimeshPrimitiveManager_setMargin)(void*,float);
void(btGImpactMeshShapePart_TrimeshPrimitiveManager_get_vertex)(void*,unsigned int,void*);
int(btConvexTriangleCallback_getTriangleCount)(void*);
void(btGImpactMeshShapePart_TrimeshPrimitiveManager_setNumfaces)(void*,int);
void(btGImpactMeshShapePart_TrimeshPrimitiveManager_setScale)(void*,const void*);
void(btDiscreteCollisionDetectorInterface_ClosestPointInput_setTransformA)(void*,const void*);
int(btCompoundShapeChild_getChildShapeType)(void*);
void(btDynamicsWorld_setGravity)(void*,const void*);
int(btGImpactMeshShapePart_getVertexCount)(void*);
int(btDbvt_countLeaves)(const void*);
float(btRotationalLimitMotor2_getCurrentPosition)(void*);
void(btInternalTriangleIndexCallback_delete)(void*);
void(btMultiBody_setJointVel)(void*,int,float);
void(btWorldImporter_setVerboseMode)(void*,int);
int(btGjkPairDetector_getDegenerateSimplex)(void*);
int(btGImpactQuantizedBvh_getNodeData)(void*,int);
void(btCollisionWorld_ClosestRayResultCallback_setHitNormalWorld)(void*,const void*);
void*(btCollisionDispatcher_new)(void*);
void(btHingeConstraint_getFrameOffsetA)(void*,void*);
float(btRigidBody_btRigidBodyConstructionInfo_getAdditionalLinearDampingThresholdSqr)(void*);
void(btGjkPairDetector_setIgnoreMargin)(void*,bool);
float(btCollisionObject_getHitFraction)(void*);
void*(btHeightfieldTerrainShape_new)(int,int,const void*,float,float,float,int,int,bool);
void(btGImpactMeshShapePart_TrimeshPrimitiveManager_setIndicestype)(void*,int);
void(btDispatcherInfo_setAllowedCcdPenetration)(void*,float);
void*(btMultibodyLink_getJointFeedback)(void*);
void*(btDbvtBroadphase_getPaircache)(void*);
void(btGjkPairDetector_setMinkowskiB)(void*,const void*);
const void*(btManifoldResult_getBody1Internal)(void*);
float(btMultibodyLink_getMass)(void*);
void*(btDispatcher_allocateCollisionAlgorithm)(void*,int);
int(btDbvt_sStkNPS_getMask)(void*);
void*(btEmptyAlgorithm_CreateFunc_new)();
int(btConeTwistConstraint_getSolveTwistLimit)(void*);
void*(btOverlappingPairCache_getOverlappingPairArrayPtr)(void*);
void(btMultiBody_getParentToLocalRot)(void*,int,void*);
void(btCollisionWorld_AllHitsRayResultCallback_setRayFromWorld)(void*,const void*);
void*(btKinematicCharacterController_new)(void*,void*,float);
void(btHingeConstraint_getInfo1NonVirtual)(void*,void*);
void(btHingeConstraint_getInfo2Internal)(void*,void*,const void*,const void*,const void*,const void*);
void(btDispatcherInfo_setStepCount)(void*,int);
bool(btTranslationalLimitMotor_needApplyForce)(void*,int);
int(btHingeConstraint_getSolveLimit)(void*);
float(btHingeConstraint_getUpperLimit)(void*);
int(btIndexedMesh_getVertexType)(void*);
float(btConeTwistConstraint_getTwistLimitSign)(void*);
void(btGeneric6DofConstraint_setAngularUpperLimit)(void*,const void*);
void(btSparseSdf_delete)(void*);
int(btSoftBody_Config_getAeromodel)(void*);
void(btJointFeedback_delete)(void*);
void(btDefaultMotionState_getStartWorldTrans)(void*,void*);
void(btRotationalLimitMotor_setCurrentLimitError)(void*,float);
void(btSoftBody_Joint_getMassmatrix)(void*,void*);
void(btMultiBodySolverConstraint_getRelpos1CrossNormal)(void*,void*);
int(btCollisionObject_getInternalType)(void*);
const void*(btConvex2dConvex2dAlgorithm_getManifold)(void*);
void(btSerializer_delete)(void*);
float(btKinematicCharacterController_getAngularDamping)(void*);
float(btKinematicCharacterController_getFallSpeed)(void*);
bool(btAABB_overlapping_trans_conservative2)(void*,const void*,const void*);
void(btOptimizedBvhNode_getAabbMaxOrg)(void*,void*);
void(btTranslationalLimitMotor2_setEquilibriumPoint)(void*,const void*);
void(btMultiBody_applyDeltaVeeMultiDof2)(void*,const float*,float);
float(btKinematicCharacterController_getJumpSpeed)(void*);
int(btCollisionWorld_RayResultCallback_getCollisionFilterMask)(void*);
float(btAngularLimit_getLow)(void*);
void(btRigidBody_getCenterOfMassTransform)(void*,void*);
void(btVoronoiSimplexSolver_delete)(void*);
void*(btCollisionDispatcher_getCollisionConfiguration)(void*);
void*(btDispatcher_findAlgorithm)(void*,const void*,const void*,void*,int);
float(btKinematicCharacterController_getStepHeight)(void*);
void(btQuantizedBvhTree_quantizePoint)(void*,unsigned short*,const void*);
void(btGeneric6DofConstraint_setLimit)(void*,int,float,float);
void(btKinematicCharacterController_getUp)(void*,void*);
void(btKinematicCharacterController_setAngularVelocity)(void*,const void*);
void(btAABB_appy_transform_trans_cache)(void*,const void*);
void(btKinematicCharacterController_setJumpSpeed)(void*,float);
void*(btSoftBody_getCdbvt)(void*);
void*(btCollisionWorld_ClosestConvexResultCallback_new)(const void*,const void*);
void(btGImpactShapeInterface_rayTest)(void*,const void*,const void*,void*);
void(btWheelInfo_RaycastInfo_delete)(void*);
float(btManifoldPoint_getAppliedImpulseLateral1)(void*);
void(btQuantizedBvh_setQuantizationValues)(void*,const void*,const void*,float);
int(btDbvtBroadphase_getFixedleft)(void*);
void(btConvexTriangleCallback_setTimeStepAndCounters)(void*,float,const void*,const void*,const void*,void*);
bool(btAABB_collide_plane)(void*,const void*);
void(btAlignedObjectArray_btSoftBody_Link_push_back)(void*,void*);
int(btManifoldPoint_getIndex0)(void*);
void(btGeneric6DofSpring2Constraint_setAxis)(void*,const void*,const void*);
void*(btCompoundFromGImpact_btCreateCompoundFromGimpactShape)(const void*,float);
void*(btDbvt_getStkStack)(void*);
float(btSoftBody_Cluster_getSelfCollisionImpulseFactor)(void*);
void*(btSoftBodyWorldInfo_new)();
void*(btWorldImporter_createCapsuleShapeZ)(void*,float,float);
float(btGeneric6DofSpring2Constraint_btGetMatrixElem)(const void*,int);
void(btMultiBody_clearVelocities)(void*);
void(btDbvtNode_setData)(void*,void*);
bool(btRotationalLimitMotor2_isLimited)(void*);
void(btManifoldPoint_setDistance)(void*,float);
void(btUnionFind_allocate)(void*,int);
bool(btTranslationalLimitMotor2_isLimited)(void*,int);
void(btRotationalLimitMotor_setBounce)(void*,float);
float(btWheelInfo_getSuspensionRestLength1)(void*);
void(btManifoldPoint_setPositionWorldOnB)(void*,const void*);
void*(btSoftBody_Impulse_operator_n)(void*);
void*(btTranslationalLimitMotor2_new)();
bool(btAABB_collide_triangle_exact)(void*,const void*,const void*,const void*,const void*);
void(btManifoldPoint_setAppliedImpulseLateral2)(void*,float);
const void*(btManifoldResult_getBody1Wrap)(void*);
bool(btRotationalLimitMotor_getEnableMotor)(void*);
void(btManifoldResult_setBody0Wrap)(void*,const void*);
void*(btRigidBody_btRigidBodyConstructionInfo_new2)(float,void*,void*,const void*);
void(btMinkowskiSumShape_getTransformA)(void*,void*);
unsigned short(btAxisSweep3_getNumHandles)(void*);
void(btCollisionAlgorithmConstructionInfo_delete)(void*);
void*(btMultiBody_new)(int,float,const void*,bool,bool);
float(btTranslationalLimitMotor_getLimitSoftness)(void*);
float(btWheelInfo_getWheelsSuspensionForce)(void*);
void(btConvexPolyhedron_project)(void*,const void*,const void*,float*,float*,void*,void*);
void(btDynamicsWorld_setInternalTickCallback)(void*,void*,void*,bool);
void*(btAxisSweep3_new)(const void*,const void*,unsigned short,void*,bool);
void(btMultiBody_addBaseTorque)(void*,const void*);
void(btMultiBody_goToSleep)(void*);
void(btMultiBody_addLinkForce)(void*,int,const void*);
bool(btBulletWorldImporter_loadFileFromMemory2)(void*,void*);
int(btAlignedObjectArray_btCollisionObjectPtr_size)(void*);
void(btMultiBody_getAngularMomentum)(void*,void*);
int(btDbvt_sStkNP_getMask)(void*);
int(btGImpactCollisionAlgorithm_getFace0)(void*);
void(btMultiBody_getBaseTorque)(void*,void*);
void(btMultiBody_getBaseWorldTransform)(void*,void*);
float(btMultiBody_getJointPos)(void*,int);
void(btIndexedMesh_setVertexBase)(void*,const unsigned char*);
float*(btMultiBody_getJointVelMultiDof)(void*,int);
float(btMultiBody_getLinkMass)(void*,int);
void(btMultiBodyJointMotor_setPositionTarget)(void*,float);
void(btDbvt_ICollide_delete)(void*);
void(btKinematicCharacterController_applyImpulse)(void*,const void*);
void(btAlignedObjectArray_btCollisionObjectPtr_resizeNoInitialize)(void*,int);
void(btSphereShape_setUnscaledRadius)(void*,float);
int(btMultiBody_getNumLinks)(void*);
void(btPoint2PointConstraint_setPivotB)(void*,const void*);
void(btGeneric6DofConstraint_calculateTransforms)(void*,const void*,const void*);
void(btMultiBodyFixedConstraint_setPivotInB)(void*,const void*);
void(btMultiBodyDynamicsWorld_addMultiBody)(void*,void*,int,int);
void*(btConvexPolyhedron_getUniqueEdges)(void*);
void(btMultiBodyDynamicsWorld_addMultiBodyConstraint)(void*,void*);
void(btMultiBodyFixedConstraint_getFrameInB)(void*,void*);
void(btRotationalLimitMotor2_setEnableSpring)(void*,bool);
void(btCollisionWorld_LocalConvexResult_setHitCollisionObject)(void*,const void*);
void(btHingeConstraint_testLimit)(void*,const void*,const void*);
void(btTranslationalLimitMotor_getCurrentLinearDiff)(void*,void*);
void*(btConvexHullShape_new)();
]])
local CLIB = ffi.load(_G.FFI_LIB or "bullet")
local library = {}
library = {
	PolyhedralConvexAabbCachingShape_recalcLocalAabb = CLIB.btPolyhedralConvexAabbCachingShape_recalcLocalAabb,
	BvhTriangleMeshShape_setOptimizedBvh = CLIB.btBvhTriangleMeshShape_setOptimizedBvh,
	EmptyAlgorithm_new = CLIB.btEmptyAlgorithm_new,
	DefaultCollisionConfiguration_new = CLIB.btDefaultCollisionConfiguration_new,
	TranslationalLimitMotor2_getSpringStiffness = CLIB.btTranslationalLimitMotor2_getSpringStiffness,
	SoftBody_setTimeacc = CLIB.btSoftBody_setTimeacc,
	SoftBody_Note_getCoords = CLIB.btSoftBody_Note_getCoords,
	SoftBody_getNotes = CLIB.btSoftBody_getNotes,
	SliderConstraint_new = CLIB.btSliderConstraint_new,
	AABB_appy_transform = CLIB.btAABB_appy_transform,
	Convex2dConvex2dAlgorithm_CreateFunc_getMinimumPointsPerturbationThreshold = CLIB.btConvex2dConvex2dAlgorithm_CreateFunc_getMinimumPointsPerturbationThreshold,
	RotationalLimitMotor_setLimitSoftness = CLIB.btRotationalLimitMotor_setLimitSoftness,
	RigidBody_computeGyroscopicImpulseImplicit_World = CLIB.btRigidBody_computeGyroscopicImpulseImplicit_World,
	RotationalLimitMotor2_getSpringStiffnessLimited = CLIB.btRotationalLimitMotor2_getSpringStiffnessLimited,
	GImpactCollisionAlgorithm_gimpact_vs_concave = CLIB.btGImpactCollisionAlgorithm_gimpact_vs_concave,
	TriangleIndexVertexArray_getIndexedMeshArray = CLIB.btTriangleIndexVertexArray_getIndexedMeshArray,
	CollisionConfiguration_delete = CLIB.btCollisionConfiguration_delete,
	CollisionWorld_LocalConvexResult_getHitCollisionObject = CLIB.btCollisionWorld_LocalConvexResult_getHitCollisionObject,
	MultiBodyFixedConstraint_getPivotInB = CLIB.btMultiBodyFixedConstraint_getPivotInB,
	CollisionObject_setWorldArrayIndex = CLIB.btCollisionObject_setWorldArrayIndex,
	CollisionWorld_LocalConvexResult_delete = CLIB.btCollisionWorld_LocalConvexResult_delete,
	PrimitiveManagerBase_get_primitive_triangle = CLIB.btPrimitiveManagerBase_get_primitive_triangle,
	ContinuousConvexCollision_new = CLIB.btContinuousConvexCollision_new,
	CollisionObject_isActive = CLIB.btCollisionObject_isActive,
	RotationalLimitMotor2_getCurrentLimit = CLIB.btRotationalLimitMotor2_getCurrentLimit,
	JointFeedback_getAppliedTorqueBodyB = CLIB.btJointFeedback_getAppliedTorqueBodyB,
	RotationalLimitMotor2_getServoTarget = CLIB.btRotationalLimitMotor2_getServoTarget,
	GImpactCollisionAlgorithm_internalGetResultOut = CLIB.btGImpactCollisionAlgorithm_internalGetResultOut,
	Dbvt_sStkNPS_delete = CLIB.btDbvt_sStkNPS_delete,
	BroadphaseInterface_rayTest2 = CLIB.btBroadphaseInterface_rayTest2,
	HingeConstraint_getInfo2InternalUsingFrameOffset = CLIB.btHingeConstraint_getInfo2InternalUsingFrameOffset,
	CollisionWorld_LocalRayResult_getHitNormalLocal = CLIB.btCollisionWorld_LocalRayResult_getHitNormalLocal,
	ManifoldPoint_setAppliedImpulseLateral1 = CLIB.btManifoldPoint_setAppliedImpulseLateral1,
	SoftBody_getTetraVertexNormalData2 = CLIB.btSoftBody_getTetraVertexNormalData2,
	CollisionWorld_contactPairTest = CLIB.btCollisionWorld_contactPairTest,
	MLCPSolver_setMLCPSolver = CLIB.btMLCPSolver_setMLCPSolver,
	OverlappingPairCache_setInternalGhostPairCallback = CLIB.btOverlappingPairCache_setInternalGhostPairCallback,
	UnionFind_new = CLIB.btUnionFind_new,
	MultiBodySolverConstraint_getJacAindex = CLIB.btMultiBodySolverConstraint_getJacAindex,
	MultiSphereShape_getSphereRadius = CLIB.btMultiSphereShape_getSphereRadius,
	TranslationalLimitMotor_getAccumulatedImpulse = CLIB.btTranslationalLimitMotor_getAccumulatedImpulse,
	MultiBody_setBaseInertia = CLIB.btMultiBody_setBaseInertia,
	CollisionShape_isInfinite = CLIB.btCollisionShape_isInfinite,
	ConeTwistConstraint_new2 = CLIB.btConeTwistConstraint_new2,
	TriangleIndexVertexArray_new = CLIB.btTriangleIndexVertexArray_new,
	Dbvt_clone = CLIB.btDbvt_clone,
	StridingMeshInterface_unLockReadOnlyVertexBase = CLIB.btStridingMeshInterface_unLockReadOnlyVertexBase,
	TetrahedronShapeEx_setVertices = CLIB.btTetrahedronShapeEx_setVertices,
	ContactSolverInfoData_getSingleAxisRollingFrictionThreshold = CLIB.btContactSolverInfoData_getSingleAxisRollingFrictionThreshold,
	CollisionShape_getUserPointer = CLIB.btCollisionShape_getUserPointer,
	BvhTriangleMeshShape_new2 = CLIB.btBvhTriangleMeshShape_new2,
	CollisionWorld_RayResultCallback_setCollisionFilterMask = CLIB.btCollisionWorld_RayResultCallback_setCollisionFilterMask,
	SoftBody_Feature_getMaterial = CLIB.btSoftBody_Feature_getMaterial,
	Generic6DofSpring2Constraint_setLimitReversed = CLIB.btGeneric6DofSpring2Constraint_setLimitReversed,
	ManifoldResult_getBody0Wrap = CLIB.btManifoldResult_getBody0Wrap,
	QuantizedBvh_getLeafNodeArray = CLIB.btQuantizedBvh_getLeafNodeArray,
	CollisionAlgorithmCreateFunc_CreateCollisionAlgorithm = CLIB.btCollisionAlgorithmCreateFunc_CreateCollisionAlgorithm,
	SliderConstraint_getSoftnessDirLin = CLIB.btSliderConstraint_getSoftnessDirLin,
	ConvexConvexAlgorithm_getManifold = CLIB.btConvexConvexAlgorithm_getManifold,
	RotationalLimitMotor2_getTargetVelocity = CLIB.btRotationalLimitMotor2_getTargetVelocity,
	GImpactBvh_get_node_pointer = CLIB.btGImpactBvh_get_node_pointer,
	RotationalLimitMotor2_setSpringStiffnessLimited = CLIB.btRotationalLimitMotor2_setSpringStiffnessLimited,
	SoftBody_Body_applyAImpulse = CLIB.btSoftBody_Body_applyAImpulse,
	BroadphaseProxy_setClientObject = CLIB.btBroadphaseProxy_setClientObject,
	SoftBody_getTotalMass = CLIB.btSoftBody_getTotalMass,
	DispatcherInfo_setTimeOfImpact = CLIB.btDispatcherInfo_setTimeOfImpact,
	SoftBody_Config_getKSK_SPLT_CL = CLIB.btSoftBody_Config_getKSK_SPLT_CL,
	SoftBody_Config_getMaxvolume = CLIB.btSoftBody_Config_getMaxvolume,
	MotionStateWrapper_new = CLIB.btMotionStateWrapper_new,
	AlignedObjectArray_btCollisionObjectPtr_at = CLIB.btAlignedObjectArray_btCollisionObjectPtr_at,
	ManifoldPoint_getUserPersistentData = CLIB.btManifoldPoint_getUserPersistentData,
	RotationalLimitMotor2_new = CLIB.btRotationalLimitMotor2_new,
	RotationalLimitMotor2_setSpringDamping = CLIB.btRotationalLimitMotor2_setSpringDamping,
	TranslationalLimitMotor2_getEquilibriumPoint = CLIB.btTranslationalLimitMotor2_getEquilibriumPoint,
	ConvexTriangleCallback_getAabbMax = CLIB.btConvexTriangleCallback_getAabbMax,
	HeightfieldTerrainShape_setUseDiamondSubdivision2 = CLIB.btHeightfieldTerrainShape_setUseDiamondSubdivision2,
	HingeConstraint_setLimit2 = CLIB.btHingeConstraint_setLimit2,
	ManifoldPoint_getCombinedContactDamping1 = CLIB.btManifoldPoint_getCombinedContactDamping1,
	PersistentManifold_new2 = CLIB.btPersistentManifold_new2,
	Generic6DofConstraint_setFrames = CLIB.btGeneric6DofConstraint_setFrames,
	CollisionObject_setActivationState = CLIB.btCollisionObject_setActivationState,
	SoftBody_Joint_getDelete = CLIB.btSoftBody_Joint_getDelete,
	SoftBody_Body_linearVelocity = CLIB.btSoftBody_Body_linearVelocity,
	TranslationalLimitMotor_getStopERP = CLIB.btTranslationalLimitMotor_getStopERP,
	SoftBody_getNodes = CLIB.btSoftBody_getNodes,
	CollisionAlgorithmCreateFunc_new = CLIB.btCollisionAlgorithmCreateFunc_new,
	BroadphaseInterface_destroyProxy = CLIB.btBroadphaseInterface_destroyProxy,
	DefaultCollisionConstructionInfo_new = CLIB.btDefaultCollisionConstructionInfo_new,
	CollisionWorld_RayResultCallback_setFlags = CLIB.btCollisionWorld_RayResultCallback_setFlags,
	CollisionWorld_AllHitsRayResultCallback_getCollisionObjects = CLIB.btCollisionWorld_AllHitsRayResultCallback_getCollisionObjects,
	ConvexShape_getNumPreferredPenetrationDirections = CLIB.btConvexShape_getNumPreferredPenetrationDirections,
	SoftBody_Impulse_operator_m = CLIB.btSoftBody_Impulse_operator_m,
	ConvexPolyhedron_initialize = CLIB.btConvexPolyhedron_initialize,
	DbvtAabbMm_Maxs = CLIB.btDbvtAabbMm_Maxs,
	SliderConstraint_getFlags = CLIB.btSliderConstraint_getFlags,
	MultiBodyJointMotor_new = CLIB.btMultiBodyJointMotor_new,
	AABB_increment_margin = CLIB.btAABB_increment_margin,
	DbvtNodePtr_array_index_of = CLIB.btDbvtNodePtr_array_index_of,
	CapsuleShape_deSerializeFloat = CLIB.btCapsuleShape_deSerializeFloat,
	SoftBody_Node_setN = CLIB.btSoftBody_Node_setN,
	CollisionWorld_delete = CLIB.btCollisionWorld_delete,
	Dbvt_update3 = CLIB.btDbvt_update3,
	ConvexPolyhedron_new = CLIB.btConvexPolyhedron_new,
	BvhTriangleMeshShape_serializeSingleTriangleInfoMap = CLIB.btBvhTriangleMeshShape_serializeSingleTriangleInfoMap,
	HingeConstraint_getLimitBiasFactor = CLIB.btHingeConstraint_getLimitBiasFactor,
	TriangleShapeEx_buildTriPlane = CLIB.btTriangleShapeEx_buildTriPlane,
	HingeConstraint_getLowerLimit = CLIB.btHingeConstraint_getLowerLimit,
	Dbvt_getFree = CLIB.btDbvt_getFree,
	CollisionObject_setAnisotropicFriction = CLIB.btCollisionObject_setAnisotropicFriction,
	Point2PointConstraint_setPivotA = CLIB.btPoint2PointConstraint_setPivotA,
	DbvtAabbMm_new = CLIB.btDbvtAabbMm_new,
	RigidBody_wantsSleeping = CLIB.btRigidBody_wantsSleeping,
	ConvexPointCloudShape_new = CLIB.btConvexPointCloudShape_new,
	ConvexHullShape_getScaledPoint = CLIB.btConvexHullShape_getScaledPoint,
	ContactSolverInfoData_setFriction = CLIB.btContactSolverInfoData_setFriction,
	SequentialImpulseConstraintSolver_btRandInt2 = CLIB.btSequentialImpulseConstraintSolver_btRandInt2,
	DbvtNode_setParent = CLIB.btDbvtNode_setParent,
	GearConstraint_getAxisA = CLIB.btGearConstraint_getAxisA,
	VoronoiSimplexSolver_getSimplex = CLIB.btVoronoiSimplexSolver_getSimplex,
	DefaultMotionState_setCenterOfMassOffset = CLIB.btDefaultMotionState_setCenterOfMassOffset,
	ConeTwistConstraint_setFrames = CLIB.btConeTwistConstraint_setFrames,
	BvhTree_clearNodes = CLIB.btBvhTree_clearNodes,
	SubSimplexClosestResult_new = CLIB.btSubSimplexClosestResult_new,
	Chunk_getNumber = CLIB.btChunk_getNumber,
	AABB_invalidate = CLIB.btAABB_invalidate,
	GImpactMeshShapePart_TrimeshPrimitiveManager_get_vertex_count = CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_get_vertex_count,
	TypedConstraint_btConstraintInfo2_getErp = CLIB.btTypedConstraint_btConstraintInfo2_getErp,
	GjkPairDetector_setDegenerateSimplex = CLIB.btGjkPairDetector_setDegenerateSimplex,
	DispatcherInfo_getTimeStep = CLIB.btDispatcherInfo_getTimeStep,
	BvhTree_getNodeBound = CLIB.btBvhTree_getNodeBound,
	SoftBody_RayFromToCaster_getTests = CLIB.btSoftBody_RayFromToCaster_getTests,
	DefaultCollisionConstructionInfo_setUseEpaPenetrationAlgorithm = CLIB.btDefaultCollisionConstructionInfo_setUseEpaPenetrationAlgorithm,
	SoftBody_getAabb = CLIB.btSoftBody_getAabb,
	PersistentManifold_getIndex1a = CLIB.btPersistentManifold_getIndex1a,
	SoftBody_getWorldInfo = CLIB.btSoftBody_getWorldInfo,
	RigidBody_setLinearVelocity = CLIB.btRigidBody_setLinearVelocity,
	RigidBody_getCenterOfMassPosition = CLIB.btRigidBody_getCenterOfMassPosition,
	Dbvt_setRoot = CLIB.btDbvt_setRoot,
	MultiBody_isPosUpdated = CLIB.btMultiBody_isPosUpdated,
	OptimizedBvhNode_getSubPart = CLIB.btOptimizedBvhNode_getSubPart,
	CollisionAlgorithmConstructionInfo_new = CLIB.btCollisionAlgorithmConstructionInfo_new,
	DynamicsWorld_addConstraint = CLIB.btDynamicsWorld_addConstraint,
	GImpactCollisionAlgorithm_getFace1 = CLIB.btGImpactCollisionAlgorithm_getFace1,
	CollisionWorld_ConvexResultCallback_setCollisionFilterMask = CLIB.btCollisionWorld_ConvexResultCallback_setCollisionFilterMask,
	MultiBody_getCanSleep = CLIB.btMultiBody_getCanSleep,
	SliderConstraint_getSoftnessOrthoAng = CLIB.btSliderConstraint_getSoftnessOrthoAng,
	TypedConstraint_getUid = CLIB.btTypedConstraint_getUid,
	MultiBodyConstraintSolver_solveMultiBodyGroup = CLIB.btMultiBodyConstraintSolver_solveMultiBodyGroup,
	Generic6DofConstraint_getInfo1NonVirtual = CLIB.btGeneric6DofConstraint_getInfo1NonVirtual,
	Chunk_new = CLIB.btChunk_new,
	Dbvt_getOpath = CLIB.btDbvt_getOpath,
	CollisionWorld_ContactResultCallback_setClosestDistanceThreshold = CLIB.btCollisionWorld_ContactResultCallback_setClosestDistanceThreshold,
	BU_Simplex1to4_getIndex = CLIB.btBU_Simplex1to4_getIndex,
	AABB_projection_interval = CLIB.btAABB_projection_interval,
	ContactSolverInfoData_setSolverMode = CLIB.btContactSolverInfoData_setSolverMode,
	SubSimplexClosestResult_setBarycentricCoordinates4 = CLIB.btSubSimplexClosestResult_setBarycentricCoordinates4,
	RigidBody_btRigidBodyConstructionInfo_setAdditionalLinearDampingThresholdSqr = CLIB.btRigidBody_btRigidBodyConstructionInfo_setAdditionalLinearDampingThresholdSqr,
	TranslationalLimitMotor_solveLinearAxis = CLIB.btTranslationalLimitMotor_solveLinearAxis,
	RigidBody_btRigidBodyConstructionInfo_getAdditionalAngularDampingThresholdSqr = CLIB.btRigidBody_btRigidBodyConstructionInfo_getAdditionalAngularDampingThresholdSqr,
	DefaultMotionState_setGraphicsWorldTrans = CLIB.btDefaultMotionState_setGraphicsWorldTrans,
	ConeTwistConstraint_getRelaxationFactor = CLIB.btConeTwistConstraint_getRelaxationFactor,
	Generic6DofSpring2Constraint_getLinearLowerLimit = CLIB.btGeneric6DofSpring2Constraint_getLinearLowerLimit,
	SoftBody_SContact_getFace = CLIB.btSoftBody_SContact_getFace,
	SoftBody_Cluster_setContainsAnchor = CLIB.btSoftBody_Cluster_setContainsAnchor,
	UsageBitfield_getUnused3 = CLIB.btUsageBitfield_getUnused3,
	ContactSolverInfoData_getTimeStep = CLIB.btContactSolverInfoData_getTimeStep,
	QuantizedBvh_reportRayOverlappingNodex = CLIB.btQuantizedBvh_reportRayOverlappingNodex,
	CollisionObject_getCollisionFlags = CLIB.btCollisionObject_getCollisionFlags,
	DiscreteDynamicsWorld_synchronizeSingleMotionState = CLIB.btDiscreteDynamicsWorld_synchronizeSingleMotionState,
	OverlappingPairCache_hasDeferredRemoval = CLIB.btOverlappingPairCache_hasDeferredRemoval,
	CollisionWorld_RayResultCallback_setCollisionFilterGroup = CLIB.btCollisionWorld_RayResultCallback_setCollisionFilterGroup,
	BroadphaseProxy_setAabbMax = CLIB.btBroadphaseProxy_setAabbMax,
	MultiBody_wakeUp = CLIB.btMultiBody_wakeUp,
	SoftBody_RContact_getCti = CLIB.btSoftBody_RContact_getCti,
	Dbvt_IClone_CloneLeaf = CLIB.btDbvt_IClone_CloneLeaf,
	UnionFind_delete = CLIB.btUnionFind_delete,
	CollisionWorld_setBroadphase = CLIB.btCollisionWorld_setBroadphase,
	MaterialProperties_getTriangleMaterialsBase = CLIB.btMaterialProperties_getTriangleMaterialsBase,
	AABB_has_collision = CLIB.btAABB_has_collision,
	CollisionObject_setInterpolationAngularVelocity = CLIB.btCollisionObject_setInterpolationAngularVelocity,
	Generic6DofSpring2Constraint_setFrames = CLIB.btGeneric6DofSpring2Constraint_setFrames,
	SoftBody_randomizeConstraints = CLIB.btSoftBody_randomizeConstraints,
	BroadphasePair_setPProxy0 = CLIB.btBroadphasePair_setPProxy0,
	Dbvt_maxdepth = CLIB.btDbvt_maxdepth,
	SoftBody_getLinkVertexData = CLIB.btSoftBody_getLinkVertexData,
	Generic6DofSpring2Constraint_getAxis = CLIB.btGeneric6DofSpring2Constraint_getAxis,
	BU_Simplex1to4_new = CLIB.btBU_Simplex1to4_new,
	AlignedObjectArray_btBroadphasePair_push_back = CLIB.btAlignedObjectArray_btBroadphasePair_push_back,
	OptimizedBvh_build = CLIB.btOptimizedBvh_build,
	Hinge2Constraint_getAnchor = CLIB.btHinge2Constraint_getAnchor,
	CollisionShape_isConvex2d = CLIB.btCollisionShape_isConvex2d,
	Generic6DofConstraint_getAngle = CLIB.btGeneric6DofConstraint_getAngle,
	QuantizedBvhTree_testQuantizedBoxOverlapp = CLIB.btQuantizedBvhTree_testQuantizedBoxOverlapp,
	HingeConstraint_setMotorTarget = CLIB.btHingeConstraint_setMotorTarget,
	SoftBody_RContact_getC4 = CLIB.btSoftBody_RContact_getC4,
	AlignedObjectArray_btSoftBody_Tetra_size = CLIB.btAlignedObjectArray_btSoftBody_Tetra_size,
	SoftBody_Impulse_delete = CLIB.btSoftBody_Impulse_delete,
	GImpactBvh_getEscapeNodeIndex = CLIB.btGImpactBvh_getEscapeNodeIndex,
	SliderConstraint_getDampingLimLin = CLIB.btSliderConstraint_getDampingLimLin,
	DefaultCollisionConfiguration_new2 = CLIB.btDefaultCollisionConfiguration_new2,
	SoftRigidDynamicsWorld_new = CLIB.btSoftRigidDynamicsWorld_new,
	SliderConstraint_getUseLinearReferenceFrameA = CLIB.btSliderConstraint_getUseLinearReferenceFrameA,
	ManifoldPoint_getCombinedRollingFriction = CLIB.btManifoldPoint_getCombinedRollingFriction,
	HingeConstraint_getFrameOffsetB = CLIB.btHingeConstraint_getFrameOffsetB,
	CollisionShape_serializeSingleShape = CLIB.btCollisionShape_serializeSingleShape,
	QuantizedBvhTree_delete = CLIB.btQuantizedBvhTree_delete,
	DbvtBroadphase_setUpdates_ratio = CLIB.btDbvtBroadphase_setUpdates_ratio,
	TriangleMesh_getUse4componentVertices = CLIB.btTriangleMesh_getUse4componentVertices,
	CollisionObject_internalSetExtensionPointer = CLIB.btCollisionObject_internalSetExtensionPointer,
	DbvtBroadphase_setStageCurrent = CLIB.btDbvtBroadphase_setStageCurrent,
	Generic6DofSpring2Constraint_enableSpring = CLIB.btGeneric6DofSpring2Constraint_enableSpring,
	TriangleShapeEx_new2 = CLIB.btTriangleShapeEx_new2,
	RotationalLimitMotor2_getBounce = CLIB.btRotationalLimitMotor2_getBounce,
	DbvtBroadphase_setVelocityPrediction = CLIB.btDbvtBroadphase_setVelocityPrediction,
	Generic6DofConstraint_new2 = CLIB.btGeneric6DofConstraint_new2,
	MultiBodyConstraint_jacobianA = CLIB.btMultiBodyConstraint_jacobianA,
	BoxBoxCollisionAlgorithm_new2 = CLIB.btBoxBoxCollisionAlgorithm_new2,
	SliderConstraint_getSoftnessOrthoLin = CLIB.btSliderConstraint_getSoftnessOrthoLin,
	Generic6DofConstraint_get_limit_motor_info2 = CLIB.btGeneric6DofConstraint_get_limit_motor_info2,
	NNCGConstraintSolver_setOnlyForNoneContact = CLIB.btNNCGConstraintSolver_setOnlyForNoneContact,
	Dbvt_sStkNP_new = CLIB.btDbvt_sStkNP_new,
	AlignedObjectArray_btSoftBody_Tetra_resizeNoInitialize = CLIB.btAlignedObjectArray_btSoftBody_Tetra_resizeNoInitialize,
	SubSimplexClosestResult_setBarycentricCoordinates3 = CLIB.btSubSimplexClosestResult_setBarycentricCoordinates3,
	QuantizedBvh_delete = CLIB.btQuantizedBvh_delete,
	Generic6DofConstraint_setAxis = CLIB.btGeneric6DofConstraint_setAxis,
	DiscreteCollisionDetectorInterface_ClosestPointInput_getTransformB = CLIB.btDiscreteCollisionDetectorInterface_ClosestPointInput_getTransformB,
	CollisionShape_serialize = CLIB.btCollisionShape_serialize,
	CompoundCollisionAlgorithm_SwappedCreateFunc_new = CLIB.btCompoundCollisionAlgorithm_SwappedCreateFunc_new,
	CollisionWorld_ClosestConvexResultCallback_setHitCollisionObject = CLIB.btCollisionWorld_ClosestConvexResultCallback_setHitCollisionObject,
	CollisionWorld_rayTest = CLIB.btCollisionWorld_rayTest,
	ConvexPolyhedron_getVertices = CLIB.btConvexPolyhedron_getVertices,
	GImpactQuantizedBvh_getEscapeNodeIndex = CLIB.btGImpactQuantizedBvh_getEscapeNodeIndex,
	DispatcherInfo_getStepCount = CLIB.btDispatcherInfo_getStepCount,
	CylinderShape_new = CLIB.btCylinderShape_new,
	SoftBody_Impulse_getAsDrift = CLIB.btSoftBody_Impulse_getAsDrift,
	StridingMeshInterface_calculateAabbBruteForce = CLIB.btStridingMeshInterface_calculateAabbBruteForce,
	RotationalLimitMotor_setAccumulatedImpulse = CLIB.btRotationalLimitMotor_setAccumulatedImpulse,
	ConvexHullShape_new2 = CLIB.btConvexHullShape_new2,
	SubSimplexClosestResult_isValid = CLIB.btSubSimplexClosestResult_isValid,
	RigidBody_new = CLIB.btRigidBody_new,
	AngularLimit_getBiasFactor = CLIB.btAngularLimit_getBiasFactor,
	RigidBody_btRigidBodyConstructionInfo_getAdditionalAngularDampingFactor = CLIB.btRigidBody_btRigidBodyConstructionInfo_getAdditionalAngularDampingFactor,
	BroadphaseProxy_setAabbMin = CLIB.btBroadphaseProxy_setAabbMin,
	OverlappingPairCache_getOverlappingPairArray = CLIB.btOverlappingPairCache_getOverlappingPairArray,
	OverlappingPairCallback_delete = CLIB.btOverlappingPairCallback_delete,
	MultiBody_getBaseCollider = CLIB.btMultiBody_getBaseCollider,
	SoftBody_CJoint_getMaxlife = CLIB.btSoftBody_CJoint_getMaxlife,
	MultiBodyJointMotor_new2 = CLIB.btMultiBodyJointMotor_new2,
	SoftBody_Cluster_getIdmass = CLIB.btSoftBody_Cluster_getIdmass,
	SoftBody_Config_getKDG = CLIB.btSoftBody_Config_getKDG,
	GImpactMeshShapePart_TrimeshPrimitiveManager_getIndexbase = CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_getIndexbase,
	RigidBody_setMotionState = CLIB.btRigidBody_setMotionState,
	Box2dShape_getPlaneEquation = CLIB.btBox2dShape_getPlaneEquation,
	RotationalLimitMotor2_setCurrentPosition = CLIB.btRotationalLimitMotor2_setCurrentPosition,
	GImpactMeshShapePart_TrimeshPrimitiveManager_getIndicestype = CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_getIndicestype,
	UniversalConstraint_getAngle1 = CLIB.btUniversalConstraint_getAngle1,
	SoftRigidDynamicsWorld_removeSoftBody = CLIB.btSoftRigidDynamicsWorld_removeSoftBody,
	GjkPairDetector_setCurIter = CLIB.btGjkPairDetector_setCurIter,
	ShapeHull_buildHull = CLIB.btShapeHull_buildHull,
	ManifoldPoint_delete = CLIB.btManifoldPoint_delete,
	AABB_collide_ray = CLIB.btAABB_collide_ray,
	GImpactCollisionAlgorithm_setPart1 = CLIB.btGImpactCollisionAlgorithm_setPart1,
	SoftBody_appendNote = CLIB.btSoftBody_appendNote,
	GjkConvexCast_new = CLIB.btGjkConvexCast_new,
	SoftBodyHelpers_DrawFrame = CLIB.btSoftBodyHelpers_DrawFrame,
	TypedConstraint_delete = CLIB.btTypedConstraint_delete,
	ManifoldPoint_setFrictionCFM = CLIB.btManifoldPoint_setFrictionCFM,
	DbvtBroadphase_getUpdates_done = CLIB.btDbvtBroadphase_getUpdates_done,
	ManifoldPoint_getCombinedFriction = CLIB.btManifoldPoint_getCombinedFriction,
	CollisionObject_setCompanionId = CLIB.btCollisionObject_setCompanionId,
	ConstraintSolver_reset = CLIB.btConstraintSolver_reset,
	ConeTwistConstraint_getSwingSpan1 = CLIB.btConeTwistConstraint_getSwingSpan1,
	AABB_copy_with_margin = CLIB.btAABB_copy_with_margin,
	MultibodyLink_getCfgOffset = CLIB.btMultibodyLink_getCfgOffset,
	SoftBody_Cluster_setCollide = CLIB.btSoftBody_Cluster_setCollide,
	RigidBody_setDamping = CLIB.btRigidBody_setDamping,
	CollisionWorld_ConvexResultCallback_needsCollision = CLIB.btCollisionWorld_ConvexResultCallback_needsCollision,
	SoftBody_CJoint_setNormal = CLIB.btSoftBody_CJoint_setNormal,
	SoftBody_Cluster_getDimpulses = CLIB.btSoftBody_Cluster_getDimpulses,
	RigidBody_btRigidBodyConstructionInfo_delete = CLIB.btRigidBody_btRigidBodyConstructionInfo_delete,
	TypedConstraint_btConstraintInfo2_getJ2linearAxis = CLIB.btTypedConstraint_btConstraintInfo2_getJ2linearAxis,
	RigidBody_btRigidBodyConstructionInfo_setLocalInertia = CLIB.btRigidBody_btRigidBodyConstructionInfo_setLocalInertia,
	Dbvt_ICollide_Process = CLIB.btDbvt_ICollide_Process,
	AlignedObjectArray_btSoftBody_MaterialPtr_size = CLIB.btAlignedObjectArray_btSoftBody_MaterialPtr_size,
	UnionFind_Free = CLIB.btUnionFind_Free,
	SoftBody_sRayCast_getFeature = CLIB.btSoftBody_sRayCast_getFeature,
	RotationalLimitMotor2_getLoLimit = CLIB.btRotationalLimitMotor2_getLoLimit,
	MultibodyLink_getCachedWorldTransform = CLIB.btMultibodyLink_getCachedWorldTransform,
	Generic6DofSpringConstraint_setDamping = CLIB.btGeneric6DofSpringConstraint_setDamping,
	MultiBody_setupSpherical = CLIB.btMultiBody_setupSpherical,
	SoftBody_Tetra_setRv = CLIB.btSoftBody_Tetra_setRv,
	Dbvt_update = CLIB.btDbvt_update,
	IndexedMesh_getIndexType = CLIB.btIndexedMesh_getIndexType,
	MultiBody_getUseGyroTerm = CLIB.btMultiBody_getUseGyroTerm,
	Generic6DofSpring2Constraint_getRotationalLimitMotor = CLIB.btGeneric6DofSpring2Constraint_getRotationalLimitMotor,
	ConvexCast_CastResult_new = CLIB.btConvexCast_CastResult_new,
	DynamicsWorld_addAction = CLIB.btDynamicsWorld_addAction,
	ConvexInternalShape_setImplicitShapeDimensions = CLIB.btConvexInternalShape_setImplicitShapeDimensions,
	SphereSphereCollisionAlgorithm_new2 = CLIB.btSphereSphereCollisionAlgorithm_new2,
	GImpactQuantizedBvh_isLeafNode = CLIB.btGImpactQuantizedBvh_isLeafNode,
	DbvtBroadphase_setPaircache = CLIB.btDbvtBroadphase_setPaircache,
	RigidBody_getOrientation = CLIB.btRigidBody_getOrientation,
	MultiBodySolverConstraint_getSolverBodyIdA = CLIB.btMultiBodySolverConstraint_getSolverBodyIdA,
	ConvexTriangleCallback_clearWrapperData = CLIB.btConvexTriangleCallback_clearWrapperData,
	DbvtNode_isleaf = CLIB.btDbvtNode_isleaf,
	RotationalLimitMotor2_getServoMotor = CLIB.btRotationalLimitMotor2_getServoMotor,
	RotationalLimitMotor2_delete = CLIB.btRotationalLimitMotor2_delete,
	SoftBody_Pose_setScl = CLIB.btSoftBody_Pose_setScl,
	SoftBody_appendAngularJoint3 = CLIB.btSoftBody_appendAngularJoint3,
	DefaultCollisionConstructionInfo_getDefaultMaxPersistentManifoldPoolSize = CLIB.btDefaultCollisionConstructionInfo_getDefaultMaxPersistentManifoldPoolSize,
	SoftBodySolver_delete = CLIB.btSoftBodySolver_delete,
	MultiBody_getJointTorqueMultiDof = CLIB.btMultiBody_getJointTorqueMultiDof,
	AlignedObjectArray_btVector3_delete = CLIB.btAlignedObjectArray_btVector3_delete,
	AlignedObjectArray_btVector3_size = CLIB.btAlignedObjectArray_btVector3_size,
	SoftBody_getFaceVertexNormalData2 = CLIB.btSoftBody_getFaceVertexNormalData2,
	AlignedObjectArray_btVector3_set = CLIB.btAlignedObjectArray_btVector3_set,
	SoftBody_Joint_Specs_setCfm = CLIB.btSoftBody_Joint_Specs_setCfm,
	AlignedObjectArray_btVector3_push_back = CLIB.btAlignedObjectArray_btVector3_push_back,
	HashedOverlappingPairCache_new = CLIB.btHashedOverlappingPairCache_new,
	AlignedObjectArray_btVector3_new = CLIB.btAlignedObjectArray_btVector3_new,
	SoftBody_getLinks = CLIB.btSoftBody_getLinks,
	SoftBody_Node_setQ = CLIB.btSoftBody_Node_setQ,
	SoftBody_Cluster_setCom = CLIB.btSoftBody_Cluster_setCom,
	Vector3_array_at = CLIB.btVector3_array_at,
	BroadphasePair_new = CLIB.btBroadphasePair_new,
	ConvexTriangleMeshShape_getMeshInterface = CLIB.btConvexTriangleMeshShape_getMeshInterface,
	SoftBodyNodePtrArray_set = CLIB.btSoftBodyNodePtrArray_set,
	SoftBodyNodePtrArray_at = CLIB.btSoftBodyNodePtrArray_at,
	MinkowskiPenetrationDepthSolver_new = CLIB.btMinkowskiPenetrationDepthSolver_new,
	BU_Simplex1to4_new4 = CLIB.btBU_Simplex1to4_new4,
	CollisionObject_isKinematicObject = CLIB.btCollisionObject_isKinematicObject,
	GImpactShapeInterface_lockChildShapes = CLIB.btGImpactShapeInterface_lockChildShapes,
	BroadphaseProxy_getUniqueId = CLIB.btBroadphaseProxy_getUniqueId,
	StridingMeshInterface_getPremadeAabb = CLIB.btStridingMeshInterface_getPremadeAabb,
	SliderConstraint_getAngularPos = CLIB.btSliderConstraint_getAngularPos,
	ManifoldPoint_getAppliedImpulse = CLIB.btManifoldPoint_getAppliedImpulse,
	ContactSolverInfo_new = CLIB.btContactSolverInfo_new,
	OptimizedBvhNode_getTriangleIndex = CLIB.btOptimizedBvhNode_getTriangleIndex,
	ConeTwistConstraint_setFixThresh = CLIB.btConeTwistConstraint_setFixThresh,
	GjkPairDetector_getCurIter = CLIB.btGjkPairDetector_getCurIter,
	OverlappingPairCache_getNumOverlappingPairs = CLIB.btOverlappingPairCache_getNumOverlappingPairs,
	CollisionObject_serializeSingleObject = CLIB.btCollisionObject_serializeSingleObject,
	DbvtBroadphase_getGid = CLIB.btDbvtBroadphase_getGid,
	MultiBodySolverConstraint_setJacAindex = CLIB.btMultiBodySolverConstraint_setJacAindex,
	SoftBody_Cluster_getMatching = CLIB.btSoftBody_Cluster_getMatching,
	MultiBodySolverConstraint_getDeltaVelBindex = CLIB.btMultiBodySolverConstraint_getDeltaVelBindex,
	CollisionWorld_ConvexResultCallback_getClosestHitFraction = CLIB.btCollisionWorld_ConvexResultCallback_getClosestHitFraction,
	HeightfieldTerrainShape_setUseZigzagSubdivision2 = CLIB.btHeightfieldTerrainShape_setUseZigzagSubdivision2,
	MultiBody_setHasSelfCollision = CLIB.btMultiBody_setHasSelfCollision,
	CollisionWorld_LocalConvexResult_setLocalShapeInfo = CLIB.btCollisionWorld_LocalConvexResult_setLocalShapeInfo,
	Hinge2Constraint_getAxis2 = CLIB.btHinge2Constraint_getAxis2,
	SliderConstraint_setUseFrameOffset = CLIB.btSliderConstraint_setUseFrameOffset,
	SoftBodyWorldInfo_delete = CLIB.btSoftBodyWorldInfo_delete,
	SoftBody_getFaceVertexData = CLIB.btSoftBody_getFaceVertexData,
	MaterialProperties_setTriangleMaterialsBase = CLIB.btMaterialProperties_setTriangleMaterialsBase,
	MaterialProperties_getMaterialType = CLIB.btMaterialProperties_getMaterialType,
	SliderConstraint_getRestitutionOrthoLin = CLIB.btSliderConstraint_getRestitutionOrthoLin,
	SparseSdf3_GarbageCollect = CLIB.btSparseSdf3_GarbageCollect,
	SoftBody_AJoint_IControl_Speed = CLIB.btSoftBody_AJoint_IControl_Speed,
	CollisionShape_getAngularMotionDisc = CLIB.btCollisionShape_getAngularMotionDisc,
	AlignedObjectArray_btSoftBody_ClusterPtr_push_back = CLIB.btAlignedObjectArray_btSoftBody_ClusterPtr_push_back,
	SoftBodyHelpers_Draw = CLIB.btSoftBodyHelpers_Draw,
	SoftBody_Anchor_getInfluence = CLIB.btSoftBody_Anchor_getInfluence,
	TypedConstraint_btConstraintInfo2_setRowskip = CLIB.btTypedConstraint_btConstraintInfo2_setRowskip,
	SoftBody_Impulse_setAsDrift = CLIB.btSoftBody_Impulse_setAsDrift,
	DbvtAabbMm_FromCE = CLIB.btDbvtAabbMm_FromCE,
	TypedConstraint_isEnabled = CLIB.btTypedConstraint_isEnabled,
	SoftBody_Joint_Specs_getErp = CLIB.btSoftBody_Joint_Specs_getErp,
	DbvtBroadphase_setUpdates_done = CLIB.btDbvtBroadphase_setUpdates_done,
	CollisionShape_isConvex = CLIB.btCollisionShape_isConvex,
	GhostObject_convexSweepTest = CLIB.btGhostObject_convexSweepTest,
	Dispatcher_getNumManifolds = CLIB.btDispatcher_getNumManifolds,
	Dbvt_getLeaves = CLIB.btDbvt_getLeaves,
	MultibodyLink_getJointType = CLIB.btMultibodyLink_getJointType,
	DbvtBroadphase_getStageCurrent = CLIB.btDbvtBroadphase_getStageCurrent,
	ManifoldResult_setPersistentManifold = CLIB.btManifoldResult_setPersistentManifold,
	CollisionWorld_LocalShapeInfo_new = CLIB.btCollisionWorld_LocalShapeInfo_new,
	Generic6DofSpring2Constraint_setAngularLowerLimitReversed = CLIB.btGeneric6DofSpring2Constraint_setAngularLowerLimitReversed,
	ConvexPlaneCollisionAlgorithm_CreateFunc_setNumPerturbationIterations = CLIB.btConvexPlaneCollisionAlgorithm_CreateFunc_setNumPerturbationIterations,
	PolyhedralConvexAabbCachingShape_getNonvirtualAabb = CLIB.btPolyhedralConvexAabbCachingShape_getNonvirtualAabb,
	ManifoldPoint_setUserPersistentData = CLIB.btManifoldPoint_setUserPersistentData,
	ConvexPointCloudShape_getNumPoints = CLIB.btConvexPointCloudShape_getNumPoints,
	SubSimplexClosestResult_getUsedVertices = CLIB.btSubSimplexClosestResult_getUsedVertices,
	MultibodyLink_getAppliedConstraintTorque = CLIB.btMultibodyLink_getAppliedConstraintTorque,
	Generic6DofSpringConstraint_isSpringEnabled = CLIB.btGeneric6DofSpringConstraint_isSpringEnabled,
	SoftBody_solveClusters = CLIB.btSoftBody_solveClusters,
	Triangle_getTriangleIndex = CLIB.btTriangle_getTriangleIndex,
	TypedConstraint_btConstraintInfo1_getNub = CLIB.btTypedConstraint_btConstraintInfo1_getNub,
	ConeShape_getHeight = CLIB.btConeShape_getHeight,
	DbvtAabbMm_Extents = CLIB.btDbvtAabbMm_Extents,
	DbvtBroadphase_setUpdates_call = CLIB.btDbvtBroadphase_setUpdates_call,
	CollisionObject_setContactStiffnessAndDamping = CLIB.btCollisionObject_setContactStiffnessAndDamping,
	SoftBody_getClusters = CLIB.btSoftBody_getClusters,
	UnionFind_unite = CLIB.btUnionFind_unite,
	SoftBodySolver_getNumberOfPositionIterations = CLIB.btSoftBodySolver_getNumberOfPositionIterations,
	ConeTwistConstraint_setMotorTargetInConstraintSpace = CLIB.btConeTwistConstraint_setMotorTargetInConstraintSpace,
	ManifoldPoint_setCombinedRestitution = CLIB.btManifoldPoint_setCombinedRestitution,
	RigidBody_getVelocityInLocalPoint = CLIB.btRigidBody_getVelocityInLocalPoint,
	TypedConstraint_getUserConstraintPtr = CLIB.btTypedConstraint_getUserConstraintPtr,
	ActionInterfaceWrapper_new = CLIB.btActionInterfaceWrapper_new,
	ConvexPolyhedron_setLocalCenter = CLIB.btConvexPolyhedron_setLocalCenter,
	AlignedObjectArray_btSoftBody_ClusterPtr_size = CLIB.btAlignedObjectArray_btSoftBody_ClusterPtr_size,
	SoftBody_Joint_setSplit = CLIB.btSoftBody_Joint_setSplit,
	ConeTwistConstraint_getSolveSwingLimit = CLIB.btConeTwistConstraint_getSolveSwingLimit,
	QuantizedBvh_calculateSerializeBufferSizeNew = CLIB.btQuantizedBvh_calculateSerializeBufferSizeNew,
	GImpactCompoundShape_addChildShape2 = CLIB.btGImpactCompoundShape_addChildShape2,
	CollisionConfiguration_getCollisionAlgorithmPool = CLIB.btCollisionConfiguration_getCollisionAlgorithmPool,
	SoftBody_Node_getLeaf = CLIB.btSoftBody_Node_getLeaf,
	Dbvt_sStkNPS_setMask = CLIB.btDbvt_sStkNPS_setMask,
	ConvexCast_CastResult_drawCoordSystem = CLIB.btConvexCast_CastResult_drawCoordSystem,
	MultiBodySolverConstraint_setAppliedPushImpulse = CLIB.btMultiBodySolverConstraint_setAppliedPushImpulse,
	ConvexTriangleMeshShape_new = CLIB.btConvexTriangleMeshShape_new,
	Dbvt_update2 = CLIB.btDbvt_update2,
	AngularLimit_getError = CLIB.btAngularLimit_getError,
	DiscreteCollisionDetectorInterface_Result_setShapeIdentifiersB = CLIB.btDiscreteCollisionDetectorInterface_Result_setShapeIdentifiersB,
	AngularLimit_fit = CLIB.btAngularLimit_fit,
	ConvexHullShape_new3 = CLIB.btConvexHullShape_new3,
	SoftBodyConcaveCollisionAlgorithm_clearCache = CLIB.btSoftBodyConcaveCollisionAlgorithm_clearCache,
	MultiBody_setupPrismatic = CLIB.btMultiBody_setupPrismatic,
	TriangleMesh_addIndex = CLIB.btTriangleMesh_addIndex,
	HingeAccumulatedAngleConstraint_new = CLIB.btHingeAccumulatedAngleConstraint_new,
	ActionInterface_delete = CLIB.btActionInterface_delete,
	BvhTriangleMeshShape_performRaycast = CLIB.btBvhTriangleMeshShape_performRaycast,
	RigidBody_btRigidBodyConstructionInfo_setMotionState = CLIB.btRigidBody_btRigidBodyConstructionInfo_setMotionState,
	CollisionWorld_ContactResultCallback_setCollisionFilterGroup = CLIB.btCollisionWorld_ContactResultCallback_setCollisionFilterGroup,
	CollisionShape_getAabb = CLIB.btCollisionShape_getAabb,
	SoftBody_appendFace2 = CLIB.btSoftBody_appendFace2,
	BroadphaseRayCallback_getLambda_max = CLIB.btBroadphaseRayCallback_getLambda_max,
	TriangleBuffer_new = CLIB.btTriangleBuffer_new,
	SliderConstraint_getCalculatedTransformA = CLIB.btSliderConstraint_getCalculatedTransformA,
	CollisionObject_getCompanionId = CLIB.btCollisionObject_getCompanionId,
	SimulationIslandManager_getUnionFind = CLIB.btSimulationIslandManager_getUnionFind,
	MultiBody_addBaseConstraintForce = CLIB.btMultiBody_addBaseConstraintForce,
	AlignedObjectArray_btIndexedMesh_size = CLIB.btAlignedObjectArray_btIndexedMesh_size,
	CollisionObject_serialize = CLIB.btCollisionObject_serialize,
	SoftBody_CJoint_setMaxlife = CLIB.btSoftBody_CJoint_setMaxlife,
	AngularLimit_new = CLIB.btAngularLimit_new,
	SoftBody_VSolve_Links = CLIB.btSoftBody_VSolve_Links,
	SoftBody_setSoftBodySolver = CLIB.btSoftBody_setSoftBodySolver,
	CompoundShapeChild_getTransform = CLIB.btCompoundShapeChild_getTransform,
	DefaultMotionState_new2 = CLIB.btDefaultMotionState_new2,
	DbvtNode_delete = CLIB.btDbvtNode_delete,
	ManifoldPoint_getNormalWorldOnB = CLIB.btManifoldPoint_getNormalWorldOnB,
	MultiBody_setCompanionId = CLIB.btMultiBody_setCompanionId,
	SoftBody_appendLinearJoint3 = CLIB.btSoftBody_appendLinearJoint3,
	MultiBody_getLinearDamping = CLIB.btMultiBody_getLinearDamping,
	PersistentManifold_getContactBreakingThreshold = CLIB.btPersistentManifold_getContactBreakingThreshold,
	CollisionObject_setDeactivationTime = CLIB.btCollisionObject_setDeactivationTime,
	CollisionObject_setInterpolationWorldTransform = CLIB.btCollisionObject_setInterpolationWorldTransform,
	CollisionShape_calculateTemporalAabb = CLIB.btCollisionShape_calculateTemporalAabb,
	RigidBody_setAngularFactor2 = CLIB.btRigidBody_setAngularFactor2,
	MultiBodySolverConstraint_getRelpos2CrossNormal = CLIB.btMultiBodySolverConstraint_getRelpos2CrossNormal,
	MinkowskiSumShape_getShapeA = CLIB.btMinkowskiSumShape_getShapeA,
	TriangleIndexVertexMaterialArray_new2 = CLIB.btTriangleIndexVertexMaterialArray_new2,
	SliderConstraint_getDampingOrthoLin = CLIB.btSliderConstraint_getDampingOrthoLin,
	RigidBody_setAngularFactor = CLIB.btRigidBody_setAngularFactor,
	CollisionAlgorithmConstructionInfo_setDispatcher1 = CLIB.btCollisionAlgorithmConstructionInfo_setDispatcher1,
	RotationalLimitMotor2_testLimitValue = CLIB.btRotationalLimitMotor2_testLimitValue,
	DispatcherInfo_setTimeStep = CLIB.btDispatcherInfo_setTimeStep,
	SliderConstraint_getFrameOffsetA = CLIB.btSliderConstraint_getFrameOffsetA,
	DbvtBroadphase_setAabbForceUpdate = CLIB.btDbvtBroadphase_setAabbForceUpdate,
	ConvexPlaneCollisionAlgorithm_CreateFunc_setMinimumPointsPerturbationThreshold = CLIB.btConvexPlaneCollisionAlgorithm_CreateFunc_setMinimumPointsPerturbationThreshold,
	ConvexTriangleCallback_getManifoldPtr = CLIB.btConvexTriangleCallback_getManifoldPtr,
	SoftBodyWorldInfo_getWater_offset = CLIB.btSoftBodyWorldInfo_getWater_offset,
	GImpactCollisionAlgorithm_getPart0 = CLIB.btGImpactCollisionAlgorithm_getPart0,
	Dbvt_update5 = CLIB.btDbvt_update5,
	CompoundShapeChild_array_at = CLIB.btCompoundShapeChild_array_at,
	BoxShape_new = CLIB.btBoxShape_new,
	GhostObject_getOverlappingObject = CLIB.btGhostObject_getOverlappingObject,
	ContactSolverInfoData_new = CLIB.btContactSolverInfoData_new,
	MultiBodySolverConstraint_getAngularComponentA = CLIB.btMultiBodySolverConstraint_getAngularComponentA,
	HeightfieldTerrainShape_new2 = CLIB.btHeightfieldTerrainShape_new2,
	BroadphaseInterface_calculateOverlappingPairs = CLIB.btBroadphaseInterface_calculateOverlappingPairs,
	MultiBodySolverConstraint_setDeltaVelAindex = CLIB.btMultiBodySolverConstraint_setDeltaVelAindex,
	MultiBody_applyDeltaVeeMultiDof = CLIB.btMultiBody_applyDeltaVeeMultiDof,
	CompoundCollisionAlgorithm_new = CLIB.btCompoundCollisionAlgorithm_new,
	CollisionObjectWrapper_getParent = CLIB.btCollisionObjectWrapper_getParent,
	GImpactBvh_getGlobalBox = CLIB.btGImpactBvh_getGlobalBox,
	DbvtBroadphase_getSets = CLIB.btDbvtBroadphase_getSets,
	RigidBody_setMassProps = CLIB.btRigidBody_setMassProps,
	SoftBody_setVolumeDensity = CLIB.btSoftBody_setVolumeDensity,
	ManifoldPoint_setCombinedFriction = CLIB.btManifoldPoint_setCombinedFriction,
	RigidBody_setAngularVelocity = CLIB.btRigidBody_setAngularVelocity,
	CollisionWorld_addCollisionObject2 = CLIB.btCollisionWorld_addCollisionObject2,
	TypedConstraint_setParam2 = CLIB.btTypedConstraint_setParam2,
	PolyhedralConvexShape_getConvexPolyhedron = CLIB.btPolyhedralConvexShape_getConvexPolyhedron,
	TypedConstraint_getUserConstraintId = CLIB.btTypedConstraint_getUserConstraintId,
	MotionState_setWorldTransform = CLIB.btMotionState_setWorldTransform,
	ContactSolverInfoData_setDamping = CLIB.btContactSolverInfoData_setDamping,
	HingeConstraint_setLimit3 = CLIB.btHingeConstraint_setLimit3,
	MultiBodySliderConstraint_new = CLIB.btMultiBodySliderConstraint_new,
	SoftBody_Material_getKAST = CLIB.btSoftBody_Material_getKAST,
	Triangle_getVertex2 = CLIB.btTriangle_getVertex2,
	GImpactCompoundShape_getCompoundPrimitiveManager = CLIB.btGImpactCompoundShape_getCompoundPrimitiveManager,
	GImpactCompoundShape_CompoundPrimitiveManager_new3 = CLIB.btGImpactCompoundShape_CompoundPrimitiveManager_new3,
	PrimitiveManagerBase_get_primitive_box = CLIB.btPrimitiveManagerBase_get_primitive_box,
	AlignedObjectArray_btPersistentManifoldPtr_size = CLIB.btAlignedObjectArray_btPersistentManifoldPtr_size,
	RigidBody_applyTorqueImpulse = CLIB.btRigidBody_applyTorqueImpulse,
	PolyhedralConvexShape_getNumPlanes = CLIB.btPolyhedralConvexShape_getNumPlanes,
	SoftBody_SolverState_setUpdmrg = CLIB.btSoftBody_SolverState_setUpdmrg,
	CollisionObject_getWorldTransform = CLIB.btCollisionObject_getWorldTransform,
	CylinderShapeX_new = CLIB.btCylinderShapeX_new,
	DbvtAabbMm_Lengths = CLIB.btDbvtAabbMm_Lengths,
	HingeConstraint_setMotorTarget2 = CLIB.btHingeConstraint_setMotorTarget2,
	MultiBodyDynamicsWorld_clearMultiBodyForces = CLIB.btMultiBodyDynamicsWorld_clearMultiBodyForces,
	CollisionObject_getInterpolationLinearVelocity = CLIB.btCollisionObject_getInterpolationLinearVelocity,
	AABB_getMax = CLIB.btAABB_getMax,
	ManifoldPoint_setIndex0 = CLIB.btManifoldPoint_setIndex0,
	MLCPSolverInterface_delete = CLIB.btMLCPSolverInterface_delete,
	MultibodyLink_getEVector = CLIB.btMultibodyLink_getEVector,
	GImpactMeshShapePart_TrimeshPrimitiveManager_getType = CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_getType,
	Dbvt_IWriter_WriteLeaf = CLIB.btDbvt_IWriter_WriteLeaf,
	TranslationalLimitMotor2_delete = CLIB.btTranslationalLimitMotor2_delete,
	MultibodyLink_getDofCount = CLIB.btMultibodyLink_getDofCount,
	AABB_delete = CLIB.btAABB_delete,
	DispatcherInfo_setDispatchFunc = CLIB.btDispatcherInfo_setDispatchFunc,
	MultiBody_stepPositionsMultiDof = CLIB.btMultiBody_stepPositionsMultiDof,
	SoftBody_AJoint_IControl_new = CLIB.btSoftBody_AJoint_IControl_new,
	VoronoiSimplexSolver_updateClosestVectorAndPoints = CLIB.btVoronoiSimplexSolver_updateClosestVectorAndPoints,
	VoronoiSimplexSolver_setNumVertices = CLIB.btVoronoiSimplexSolver_setNumVertices,
	SoftBody_ImplicitFnWrapper_new = CLIB.btSoftBody_ImplicitFnWrapper_new,
	ScaledBvhTriangleMeshShape_new = CLIB.btScaledBvhTriangleMeshShape_new,
	BvhTree_delete = CLIB.btBvhTree_delete,
	RotationalLimitMotor2_getCurrentLimitErrorHi = CLIB.btRotationalLimitMotor2_getCurrentLimitErrorHi,
	VoronoiSimplexSolver_setNeedsUpdate = CLIB.btVoronoiSimplexSolver_setNeedsUpdate,
	VoronoiSimplexSolver_setLastW = CLIB.btVoronoiSimplexSolver_setLastW,
	SoftBody_appendAngularJoint2 = CLIB.btSoftBody_appendAngularJoint2,
	SoftBody_Config_getKSS_SPLT_CL = CLIB.btSoftBody_Config_getKSS_SPLT_CL,
	VoronoiSimplexSolver_setCachedV = CLIB.btVoronoiSimplexSolver_setCachedV,
	AABB_setMax = CLIB.btAABB_setMax,
	VoronoiSimplexSolver_setCachedP2 = CLIB.btVoronoiSimplexSolver_setCachedP2,
	VoronoiSimplexSolver_setCachedP1 = CLIB.btVoronoiSimplexSolver_setCachedP1,
	GjkPairDetector_new2 = CLIB.btGjkPairDetector_new2,
	VoronoiSimplexSolver_setCachedBC = CLIB.btVoronoiSimplexSolver_setCachedBC,
	SoftBody_cutLink = CLIB.btSoftBody_cutLink,
	VoronoiSimplexSolver_reset = CLIB.btVoronoiSimplexSolver_reset,
	VoronoiSimplexSolver_removeVertex = CLIB.btVoronoiSimplexSolver_removeVertex,
	VoronoiSimplexSolver_reduceVertices = CLIB.btVoronoiSimplexSolver_reduceVertices,
	ContactSolverInfoData_getWarmstartingFactor = CLIB.btContactSolverInfoData_getWarmstartingFactor,
	AngularLimit_test = CLIB.btAngularLimit_test,
	SoftBody_defaultCollisionHandler2 = CLIB.btSoftBody_defaultCollisionHandler2,
	TranslationalLimitMotor2_getTargetVelocity = CLIB.btTranslationalLimitMotor2_getTargetVelocity,
	VoronoiSimplexSolver_pointOutsideOfPlane = CLIB.btVoronoiSimplexSolver_pointOutsideOfPlane,
	Point2PointConstraint_getSetting = CLIB.btPoint2PointConstraint_getSetting,
	VoronoiSimplexSolver_numVertices = CLIB.btVoronoiSimplexSolver_numVertices,
	VoronoiSimplexSolver_maxVertex = CLIB.btVoronoiSimplexSolver_maxVertex,
	VoronoiSimplexSolver_inSimplex = CLIB.btVoronoiSimplexSolver_inSimplex,
	Chunk_delete = CLIB.btChunk_delete,
	MultiBody_setBaseWorldTransform = CLIB.btMultiBody_setBaseWorldTransform,
	ConvexShape_batchedUnitVectorGetSupportingVertexWithoutMargin = CLIB.btConvexShape_batchedUnitVectorGetSupportingVertexWithoutMargin,
	TypedConstraint_btConstraintInfo2_getLowerLimit = CLIB.btTypedConstraint_btConstraintInfo2_getLowerLimit,
	Hinge2Constraint_getAxis1 = CLIB.btHinge2Constraint_getAxis1,
	OptimizedBvhNode_delete = CLIB.btOptimizedBvhNode_delete,
	CollisionWorld_setForceUpdateAllAabbs = CLIB.btCollisionWorld_setForceUpdateAllAabbs,
	IndexedMesh_delete = CLIB.btIndexedMesh_delete,
	VoronoiSimplexSolver_getSimplexPointsQ = CLIB.btVoronoiSimplexSolver_getSimplexPointsQ,
	BroadphaseInterface_aabbTest = CLIB.btBroadphaseInterface_aabbTest,
	AlignedObjectArray_btPersistentManifoldPtr_delete = CLIB.btAlignedObjectArray_btPersistentManifoldPtr_delete,
	VoronoiSimplexSolver_getSimplexPointsP = CLIB.btVoronoiSimplexSolver_getSimplexPointsP,
	CompoundShapeChild_getNode = CLIB.btCompoundShapeChild_getNode,
	VoronoiSimplexSolver_getNeedsUpdate = CLIB.btVoronoiSimplexSolver_getNeedsUpdate,
	SoftBody_rayTest2 = CLIB.btSoftBody_rayTest2,
	SoftBody_Cluster_setNdamping = CLIB.btSoftBody_Cluster_setNdamping,
	VoronoiSimplexSolver_getCachedValidClosest = CLIB.btVoronoiSimplexSolver_getCachedValidClosest,
	VoronoiSimplexSolver_getCachedV = CLIB.btVoronoiSimplexSolver_getCachedV,
	UniversalConstraint_getAnchor2 = CLIB.btUniversalConstraint_getAnchor2,
	MultiBody_clearConstraintForces = CLIB.btMultiBody_clearConstraintForces,
	MultiBody_setJointPos = CLIB.btMultiBody_setJointPos,
	DefaultMotionState_getCenterOfMassOffset = CLIB.btDefaultMotionState_getCenterOfMassOffset,
	ConeTwistConstraint_isPastSwingLimit = CLIB.btConeTwistConstraint_isPastSwingLimit,
	GearConstraint_new = CLIB.btGearConstraint_new,
	TranslationalLimitMotor_setUpperLimit = CLIB.btTranslationalLimitMotor_setUpperLimit,
	DynamicsWorld_setWorldUserInfo = CLIB.btDynamicsWorld_setWorldUserInfo,
	TypedConstraint_getUserConstraintType = CLIB.btTypedConstraint_getUserConstraintType,
	GImpactBvh_getLeftNode = CLIB.btGImpactBvh_getLeftNode,
	DispatcherInfo_getUseEpa = CLIB.btDispatcherInfo_getUseEpa,
	AlignedObjectArray_btSoftBody_Anchor_resizeNoInitialize = CLIB.btAlignedObjectArray_btSoftBody_Anchor_resizeNoInitialize,
	VoronoiSimplexSolver_fullSimplex = CLIB.btVoronoiSimplexSolver_fullSimplex,
	VoronoiSimplexSolver_emptySimplex = CLIB.btVoronoiSimplexSolver_emptySimplex,
	ManifoldPoint_setLateralFrictionDir2 = CLIB.btManifoldPoint_setLateralFrictionDir2,
	VoronoiSimplexSolver_compute_points = CLIB.btVoronoiSimplexSolver_compute_points,
	TranslationalLimitMotor2_getSpringDamping = CLIB.btTranslationalLimitMotor2_getSpringDamping,
	CompoundShape_getChildTransform = CLIB.btCompoundShape_getChildTransform,
	Dbvt_insert = CLIB.btDbvt_insert,
	VoronoiSimplexSolver_closestPtPointTetrahedron = CLIB.btVoronoiSimplexSolver_closestPtPointTetrahedron,
	SoftBody_Pose_getScl = CLIB.btSoftBody_Pose_getScl,
	MultiBodySolverConstraint_getContactNormal2 = CLIB.btMultiBodySolverConstraint_getContactNormal2,
	RotationalLimitMotor2_getHiLimit = CLIB.btRotationalLimitMotor2_getHiLimit,
	TranslationalLimitMotor2_setServoTarget = CLIB.btTranslationalLimitMotor2_setServoTarget,
	SoftBody_Anchor_getLocal = CLIB.btSoftBody_Anchor_getLocal,
	VoronoiSimplexSolver_backup_closest = CLIB.btVoronoiSimplexSolver_backup_closest,
	ContactSolverInfoData_setMinimumSolverBatchSize = CLIB.btContactSolverInfoData_setMinimumSolverBatchSize,
	VoronoiSimplexSolver_addVertex = CLIB.btVoronoiSimplexSolver_addVertex,
	VoronoiSimplexSolver_new = CLIB.btVoronoiSimplexSolver_new,
	SliderConstraint_getCalculatedTransformB = CLIB.btSliderConstraint_getCalculatedTransformB,
	SubSimplexClosestResult_setUsedVertices = CLIB.btSubSimplexClosestResult_setUsedVertices,
	SubSimplexClosestResult_setDegenerate = CLIB.btSubSimplexClosestResult_setDegenerate,
	ConstraintSolver_allSolved = CLIB.btConstraintSolver_allSolved,
	SubSimplexClosestResult_setClosestPointOnSimplex = CLIB.btSubSimplexClosestResult_setClosestPointOnSimplex,
	MultiBodyConstraint_internalSetAppliedImpulse = CLIB.btMultiBodyConstraint_internalSetAppliedImpulse,
	PrimitiveTriangle_new = CLIB.btPrimitiveTriangle_new,
	MultiBody_processDeltaVeeMultiDof2 = CLIB.btMultiBody_processDeltaVeeMultiDof2,
	DbvtBroadphase_setPid = CLIB.btDbvtBroadphase_setPid,
	SubSimplexClosestResult_setBarycentricCoordinates5 = CLIB.btSubSimplexClosestResult_setBarycentricCoordinates5,
	CollisionWorld_RayResultCallback_getCollisionObject = CLIB.btCollisionWorld_RayResultCallback_getCollisionObject,
	MultiBody_setUserIndex = CLIB.btMultiBody_setUserIndex,
	SubSimplexClosestResult_setBarycentricCoordinates2 = CLIB.btSubSimplexClosestResult_setBarycentricCoordinates2,
	SubSimplexClosestResult_setBarycentricCoordinates = CLIB.btSubSimplexClosestResult_setBarycentricCoordinates,
	CollisionWorld_LocalConvexResult_getLocalShapeInfo = CLIB.btCollisionWorld_LocalConvexResult_getLocalShapeInfo,
	Generic6DofConstraint_getInfo2NonVirtual = CLIB.btGeneric6DofConstraint_getInfo2NonVirtual,
	Dbvt_extractLeaves = CLIB.btDbvt_extractLeaves,
	SubSimplexClosestResult_getClosestPointOnSimplex = CLIB.btSubSimplexClosestResult_getClosestPointOnSimplex,
	SubSimplexClosestResult_getBarycentricCoords = CLIB.btSubSimplexClosestResult_getBarycentricCoords,
	Generic6DofSpring2Constraint_new = CLIB.btGeneric6DofSpring2Constraint_new,
	MultiBodyDynamicsWorld_removeMultiBodyConstraint = CLIB.btMultiBodyDynamicsWorld_removeMultiBodyConstraint,
	UsageBitfield_setUsedVertexD = CLIB.btUsageBitfield_setUsedVertexD,
	ConvexCast_CastResult_delete = CLIB.btConvexCast_CastResult_delete,
	CollisionObject_isStaticObject = CLIB.btCollisionObject_isStaticObject,
	UsageBitfield_setUsedVertexC = CLIB.btUsageBitfield_setUsedVertexC,
	DiscreteCollisionDetectorInterface_delete = CLIB.btDiscreteCollisionDetectorInterface_delete,
	MultiBody_getBasePos = CLIB.btMultiBody_getBasePos,
	BroadphaseInterface_getBroadphaseAabb = CLIB.btBroadphaseInterface_getBroadphaseAabb,
	MultiBodySolverConstraint_setCfm = CLIB.btMultiBodySolverConstraint_setCfm,
	AlignedObjectArray_btSoftBody_Tetra_at = CLIB.btAlignedObjectArray_btSoftBody_Tetra_at,
	UsageBitfield_setUnused4 = CLIB.btUsageBitfield_setUnused4,
	ConvexTriangleCallback_setManifoldPtr = CLIB.btConvexTriangleCallback_setManifoldPtr,
	TranslationalLimitMotor2_new2 = CLIB.btTranslationalLimitMotor2_new2,
	UsageBitfield_setUnused3 = CLIB.btUsageBitfield_setUnused3,
	ContactSolverInfoData_getTau = CLIB.btContactSolverInfoData_getTau,
	HingeConstraint_getHingeAngle = CLIB.btHingeConstraint_getHingeAngle,
	UsageBitfield_setUnused2 = CLIB.btUsageBitfield_setUnused2,
	UsageBitfield_setUnused1 = CLIB.btUsageBitfield_setUnused1,
	ConvexShape_localGetSupportVertexWithoutMarginNonVirtual = CLIB.btConvexShape_localGetSupportVertexWithoutMarginNonVirtual,
	PrimitiveManagerBase_is_trimesh = CLIB.btPrimitiveManagerBase_is_trimesh,
	SoftBody_Link_getC2 = CLIB.btSoftBody_Link_getC2,
	MultiBody_useGlobalVelocities = CLIB.btMultiBody_useGlobalVelocities,
	UsageBitfield_getUsedVertexD = CLIB.btUsageBitfield_getUsedVertexD,
	CollisionShape_isConcave = CLIB.btCollisionShape_isConcave,
	UsageBitfield_getUsedVertexC = CLIB.btUsageBitfield_getUsedVertexC,
	SoftBody_RayFromToCaster_getFace = CLIB.btSoftBody_RayFromToCaster_getFace,
	UsageBitfield_getUsedVertexB = CLIB.btUsageBitfield_getUsedVertexB,
	Dbvt_sStkNP_getNode = CLIB.btDbvt_sStkNP_getNode,
	MultiBodySolverConstraint_getAppliedImpulse = CLIB.btMultiBodySolverConstraint_getAppliedImpulse,
	UsageBitfield_getUnused4 = CLIB.btUsageBitfield_getUnused4,
	MultiBody_getLink = CLIB.btMultiBody_getLink,
	PolyhedralConvexShape_getPlane = CLIB.btPolyhedralConvexShape_getPlane,
	UsageBitfield_getUnused2 = CLIB.btUsageBitfield_getUnused2,
	BroadphaseRayCallbackWrapper_new = CLIB.btBroadphaseRayCallbackWrapper_new,
	UsageBitfield_getUnused1 = CLIB.btUsageBitfield_getUnused1,
	GImpactBvh_setNodeBound = CLIB.btGImpactBvh_setNodeBound,
	Dispatcher_getInternalManifoldPointer = CLIB.btDispatcher_getInternalManifoldPointer,
	MultibodyLink_getParent = CLIB.btMultibodyLink_getParent,
	DefaultMotionState_new = CLIB.btDefaultMotionState_new,
	SoftBody_RayFromToCaster_setRayTo = CLIB.btSoftBody_RayFromToCaster_setRayTo,
	HingeAccumulatedAngleConstraint_new2 = CLIB.btHingeAccumulatedAngleConstraint_new2,
	MultiBody_setupFixed = CLIB.btMultiBody_setupFixed,
	Generic6DofConstraint_getRelativePivotPosition = CLIB.btGeneric6DofConstraint_getRelativePivotPosition,
	MultiBodyConstraint_getIslandIdB = CLIB.btMultiBodyConstraint_getIslandIdB,
	DiscreteDynamicsWorld_getLatencyMotionStateInterpolation = CLIB.btDiscreteDynamicsWorld_getLatencyMotionStateInterpolation,
	UniversalConstraint_setUpperLimit = CLIB.btUniversalConstraint_setUpperLimit,
	SoftBodyWorldInfo_getWater_density = CLIB.btSoftBodyWorldInfo_getWater_density,
	UniversalConstraint_setLowerLimit = CLIB.btUniversalConstraint_setLowerLimit,
	UniversalConstraint_getAxis2 = CLIB.btUniversalConstraint_getAxis2,
	SoftBodyConcaveCollisionAlgorithm_SwappedCreateFunc_new = CLIB.btSoftBodyConcaveCollisionAlgorithm_SwappedCreateFunc_new,
	AABB_plane_classify = CLIB.btAABB_plane_classify,
	CollisionWorld_setDebugDrawer = CLIB.btCollisionWorld_setDebugDrawer,
	MinkowskiSumShape_GetTransformB = CLIB.btMinkowskiSumShape_GetTransformB,
	SoftBody_clusterCom = CLIB.btSoftBody_clusterCom,
	MultiBodyConstraint_setPosition = CLIB.btMultiBodyConstraint_setPosition,
	SoftBody_updateClusters = CLIB.btSoftBody_updateClusters,
	CollisionObject_setCollisionShape = CLIB.btCollisionObject_setCollisionShape,
	UniversalConstraint_new = CLIB.btUniversalConstraint_new,
	UnionFind_sortIslands = CLIB.btUnionFind_sortIslands,
	UnionFind_reset = CLIB.btUnionFind_reset,
	AlignedObjectArray_btSoftBody_ClusterPtr_resizeNoInitialize = CLIB.btAlignedObjectArray_btSoftBody_ClusterPtr_resizeNoInitialize,
	QuantizedBvhNode_getEscapeIndex = CLIB.btQuantizedBvhNode_getEscapeIndex,
	Box2dShape_getNormals = CLIB.btBox2dShape_getNormals,
	CollisionObject_calculateSerializeBufferSize = CLIB.btCollisionObject_calculateSerializeBufferSize,
	Dbvt_nearest = CLIB.btDbvt_nearest,
	UnionFind_getNumElements = CLIB.btUnionFind_getNumElements,
	CollisionShape_isCompound = CLIB.btCollisionShape_isCompound,
	RigidBody_getInvInertiaTensorWorld = CLIB.btRigidBody_getInvInertiaTensorWorld,
	SoftBody_addVelocity = CLIB.btSoftBody_addVelocity,
	MultiBody_hasFixedBase = CLIB.btMultiBody_hasFixedBase,
	SoftBody_RayFromToCaster_rayFromToTriangle = CLIB.btSoftBody_RayFromToCaster_rayFromToTriangle,
	MaterialProperties_setMaterialType = CLIB.btMaterialProperties_setMaterialType,
	StridingMeshInterface_preallocateIndices = CLIB.btStridingMeshInterface_preallocateIndices,
	TriangleMeshShape_getLocalAabbMax = CLIB.btTriangleMeshShape_getLocalAabbMax,
	MultiBody_setMaxCoordinateVelocity = CLIB.btMultiBody_setMaxCoordinateVelocity,
	MultiBody_stepVelocitiesMultiDof = CLIB.btMultiBody_stepVelocitiesMultiDof,
	SphereShape_new = CLIB.btSphereShape_new,
	AlignedObjectArray_btSoftBody_Face_resizeNoInitialize = CLIB.btAlignedObjectArray_btSoftBody_Face_resizeNoInitialize,
	UnionFind_find = CLIB.btUnionFind_find,
	SphereTriangleCollisionAlgorithm_new = CLIB.btSphereTriangleCollisionAlgorithm_new,
	ManifoldPoint_setLateralFrictionDir1 = CLIB.btManifoldPoint_setLateralFrictionDir1,
	Element_delete = CLIB.btElement_delete,
	Element_setSz = CLIB.btElement_setSz,
	Element_setId = CLIB.btElement_setId,
	CollisionObject_isStaticOrKinematicObject = CLIB.btCollisionObject_isStaticOrKinematicObject,
	Element_getSz = CLIB.btElement_getSz,
	Element_getId = CLIB.btElement_getId,
	DbvtBroadphase_benchmark = CLIB.btDbvtBroadphase_benchmark,
	Element_new = CLIB.btElement_new,
	StridingMeshInterface_calculateSerializeBufferSize = CLIB.btStridingMeshInterface_calculateSerializeBufferSize,
	Generic6DofSpring2Constraint_isLimited = CLIB.btGeneric6DofSpring2Constraint_isLimited,
	UniformScalingShape_getUniformScalingFactor = CLIB.btUniformScalingShape_getUniformScalingFactor,
	UniformScalingShape_getChildShape = CLIB.btUniformScalingShape_getChildShape,
	UniformScalingShape_new = CLIB.btUniformScalingShape_new,
	MultiBody_getLinkTorque = CLIB.btMultiBody_getLinkTorque,
	SoftBody_Joint_Specs_getCfm = CLIB.btSoftBody_Joint_Specs_getCfm,
	ManifoldPoint_getLateralFrictionDir2 = CLIB.btManifoldPoint_getLateralFrictionDir2,
	SequentialImpulseConstraintSolver_btRand2 = CLIB.btSequentialImpulseConstraintSolver_btRand2,
	QuantizedBvhNode_getQuantizedAabbMin = CLIB.btQuantizedBvhNode_getQuantizedAabbMin,
	ConeTwistConstraint_setLimit2 = CLIB.btConeTwistConstraint_setLimit2,
	DefaultSerializer_internalAlloc = CLIB.btDefaultSerializer_internalAlloc,
	ConvexCast_CastResult_getNormal = CLIB.btConvexCast_CastResult_getNormal,
	SoftBody_Joint_delete = CLIB.btSoftBody_Joint_delete,
	SoftBody_AJoint_Specs_getAxis = CLIB.btSoftBody_AJoint_Specs_getAxis,
	DbvtBroadphase_setGid = CLIB.btDbvtBroadphase_setGid,
	SoftBody_Body_angularVelocity2 = CLIB.btSoftBody_Body_angularVelocity2,
	ConvexPolyhedron_setMC = CLIB.btConvexPolyhedron_setMC,
	AngularLimit_getCorrection = CLIB.btAngularLimit_getCorrection,
	SliderConstraint_getSoftnessDirAng = CLIB.btSliderConstraint_getSoftnessDirAng,
	CollisionObject_getContactStiffness = CLIB.btCollisionObject_getContactStiffness,
	RigidBody_btRigidBodyConstructionInfo_new = CLIB.btRigidBody_btRigidBodyConstructionInfo_new,
	TypedConstraint_setUserConstraintType = CLIB.btTypedConstraint_setUserConstraintType,
	MultiBody_getCompanionId = CLIB.btMultiBody_getCompanionId,
	TypedConstraint_setUserConstraintPtr = CLIB.btTypedConstraint_setUserConstraintPtr,
	Dbvt_IWriter_Prepare = CLIB.btDbvt_IWriter_Prepare,
	TypedConstraint_setupSolverConstraint = CLIB.btTypedConstraint_setupSolverConstraint,
	MultiBody_getBaseName = CLIB.btMultiBody_getBaseName,
	TypedConstraint_setParam = CLIB.btTypedConstraint_setParam,
	TypedConstraint_setOverrideNumSolverIterations = CLIB.btTypedConstraint_setOverrideNumSolverIterations,
	SliderConstraint_getMaxAngMotorForce = CLIB.btSliderConstraint_getMaxAngMotorForce,
	TranslationalLimitMotor_getStopCFM = CLIB.btTranslationalLimitMotor_getStopCFM,
	GjkPairDetector_getCatchDegeneracies = CLIB.btGjkPairDetector_getCatchDegeneracies,
	TypedConstraint_setJointFeedback = CLIB.btTypedConstraint_setJointFeedback,
	GImpactQuantizedBvh_boxQueryTrans = CLIB.btGImpactQuantizedBvh_boxQueryTrans,
	TypedConstraint_setEnabled = CLIB.btTypedConstraint_setEnabled,
	ManifoldPoint_setIndex1 = CLIB.btManifoldPoint_setIndex1,
	TypedConstraint_setDbgDrawSize = CLIB.btTypedConstraint_setDbgDrawSize,
	Dbvt_IWriter_WriteNode = CLIB.btDbvt_IWriter_WriteNode,
	TypedConstraint_setBreakingImpulseThreshold = CLIB.btTypedConstraint_setBreakingImpulseThreshold,
	TypedConstraint_serialize = CLIB.btTypedConstraint_serialize,
	GjkPairDetector_new = CLIB.btGjkPairDetector_new,
	SoftBody_RContact_setC4 = CLIB.btSoftBody_RContact_setC4,
	TypedConstraint_internalSetAppliedImpulse = CLIB.btTypedConstraint_internalSetAppliedImpulse,
	Generic6DofConstraint_getAngularUpperLimit = CLIB.btGeneric6DofConstraint_getAngularUpperLimit,
	GjkPairDetector_getClosestPointsNonVirtual = CLIB.btGjkPairDetector_getClosestPointsNonVirtual,
	CylinderShape_getRadius = CLIB.btCylinderShape_getRadius,
	VoronoiSimplexSolver_getCachedBC = CLIB.btVoronoiSimplexSolver_getCachedBC,
	CollisionWorld_LocalConvexResult_new = CLIB.btCollisionWorld_LocalConvexResult_new,
	CollisionWorld_ConvexResultCallbackWrapper_needsCollision = CLIB.btCollisionWorld_ConvexResultCallbackWrapper_needsCollision,
	TypedConstraint_getRigidBodyB = CLIB.btTypedConstraint_getRigidBodyB,
	StridingMeshInterface_getLockedReadOnlyVertexIndexBase = CLIB.btStridingMeshInterface_getLockedReadOnlyVertexIndexBase,
	JointFeedback_setAppliedForceBodyA = CLIB.btJointFeedback_setAppliedForceBodyA,
	TypedConstraint_getParam2 = CLIB.btTypedConstraint_getParam2,
	MultiBody_getLinkInertia = CLIB.btMultiBody_getLinkInertia,
	DispatcherInfo_getEnableSPU = CLIB.btDispatcherInfo_getEnableSPU,
	TypedConstraint_getParam = CLIB.btTypedConstraint_getParam,
	MultiBodySolverConstraint_setLinkA = CLIB.btMultiBodySolverConstraint_setLinkA,
	SoftBody_setTotalDensity = CLIB.btSoftBody_setTotalDensity,
	SoftBody_Pose_getPos = CLIB.btSoftBody_Pose_getPos,
	TypedConstraint_getJointFeedback = CLIB.btTypedConstraint_getJointFeedback,
	TranslationalLimitMotor_getEnableMotor = CLIB.btTranslationalLimitMotor_getEnableMotor,
	MultiBody_addJointTorqueMultiDof2 = CLIB.btMultiBody_addJointTorqueMultiDof2,
	AABB_merge = CLIB.btAABB_merge,
	SoftBody_Tetra_getLeaf = CLIB.btSoftBody_Tetra_getLeaf,
	SoftBody_Tetra_getRv = CLIB.btSoftBody_Tetra_getRv,
	SoftBody_Material_getKLST = CLIB.btSoftBody_Material_getKLST,
	ConeShapeX_new = CLIB.btConeShapeX_new,
	RigidBody_getGravity = CLIB.btRigidBody_getGravity,
	TypedConstraint_getInfo2 = CLIB.btTypedConstraint_getInfo2,
	TypedConstraint_getInfo1 = CLIB.btTypedConstraint_getInfo1,
	RotationalLimitMotor_getMaxLimitForce = CLIB.btRotationalLimitMotor_getMaxLimitForce,
	ConstraintSetting_setImpulseClamp = CLIB.btConstraintSetting_setImpulseClamp,
	TypedConstraint_getConstraintType = CLIB.btTypedConstraint_getConstraintType,
	SoftBody_sRayCast_getBody = CLIB.btSoftBody_sRayCast_getBody,
	TypedConstraint_getBreakingImpulseThreshold = CLIB.btTypedConstraint_getBreakingImpulseThreshold,
	MultiBody_setBaseVel = CLIB.btMultiBody_setBaseVel,
	TypedConstraint_getAppliedImpulse = CLIB.btTypedConstraint_getAppliedImpulse,
	Dbvt_sStkNN_delete = CLIB.btDbvt_sStkNN_delete,
	TypedConstraint_calculateSerializeBufferSize = CLIB.btTypedConstraint_calculateSerializeBufferSize,
	ConeShape_setRadius = CLIB.btConeShape_setRadius,
	TypedConstraint_buildJacobian = CLIB.btTypedConstraint_buildJacobian,
	CollisionObject_getBroadphaseHandle = CLIB.btCollisionObject_getBroadphaseHandle,
	TypedConstraint_btConstraintInfo2_delete = CLIB.btTypedConstraint_btConstraintInfo2_delete,
	TypedConstraint_btConstraintInfo2_setUpperLimit = CLIB.btTypedConstraint_btConstraintInfo2_setUpperLimit,
	AlignedObjectArray_btIndexedMesh_at = CLIB.btAlignedObjectArray_btIndexedMesh_at,
	TypedConstraint_btConstraintInfo2_setNumIterations = CLIB.btTypedConstraint_btConstraintInfo2_setNumIterations,
	TypedConstraint_btConstraintInfo2_setLowerLimit = CLIB.btTypedConstraint_btConstraintInfo2_setLowerLimit,
	CollisionShape_isSoftBody = CLIB.btCollisionShape_isSoftBody,
	ConvexPointCloudShape_getScaledPoint = CLIB.btConvexPointCloudShape_getScaledPoint,
	MultiBodyConstraint_getMaxAppliedImpulse = CLIB.btMultiBodyConstraint_getMaxAppliedImpulse,
	TypedConstraint_btConstraintInfo2_setJ2linearAxis = CLIB.btTypedConstraint_btConstraintInfo2_setJ2linearAxis,
	TypedConstraint_btConstraintInfo2_setJ2angularAxis = CLIB.btTypedConstraint_btConstraintInfo2_setJ2angularAxis,
	DbvtBroadphase_getNewpairs = CLIB.btDbvtBroadphase_getNewpairs,
	MultiBody_setBasePos = CLIB.btMultiBody_setBasePos,
	MultiBody_getVelocityVector = CLIB.btMultiBody_getVelocityVector,
	TypedConstraint_btConstraintInfo2_setJ1linearAxis = CLIB.btTypedConstraint_btConstraintInfo2_setJ1linearAxis,
	TypedConstraint_btConstraintInfo2_setJ1angularAxis = CLIB.btTypedConstraint_btConstraintInfo2_setJ1angularAxis,
	TypedConstraint_btConstraintInfo2_setFps = CLIB.btTypedConstraint_btConstraintInfo2_setFps,
	GImpactBvh_delete = CLIB.btGImpactBvh_delete,
	TypedConstraint_btConstraintInfo2_setErp = CLIB.btTypedConstraint_btConstraintInfo2_setErp,
	Dispatcher_freeCollisionAlgorithm = CLIB.btDispatcher_freeCollisionAlgorithm,
	TypedConstraint_btConstraintInfo2_setDamping = CLIB.btTypedConstraint_btConstraintInfo2_setDamping,
	QuantizedBvh_isQuantized = CLIB.btQuantizedBvh_isQuantized,
	QuantizedBvh_calculateSerializeBufferSize = CLIB.btQuantizedBvh_calculateSerializeBufferSize,
	TypedConstraint_btConstraintInfo2_setConstraintError = CLIB.btTypedConstraint_btConstraintInfo2_setConstraintError,
	TypedConstraint_btConstraintInfo2_setCfm = CLIB.btTypedConstraint_btConstraintInfo2_setCfm,
	TypedConstraint_btConstraintInfo2_getUpperLimit = CLIB.btTypedConstraint_btConstraintInfo2_getUpperLimit,
	SoftBody_clusterCount = CLIB.btSoftBody_clusterCount,
	DiscreteDynamicsWorld_getCollisionWorld = CLIB.btDiscreteDynamicsWorld_getCollisionWorld,
	StridingMeshInterface_getNumSubParts = CLIB.btStridingMeshInterface_getNumSubParts,
	MultibodyLink_getFlags = CLIB.btMultibodyLink_getFlags,
	ManifoldPoint_getCombinedRestitution = CLIB.btManifoldPoint_getCombinedRestitution,
	CollisionObjectWrapper_setPartId = CLIB.btCollisionObjectWrapper_setPartId,
	ConvexCast_CastResult_DebugDraw = CLIB.btConvexCast_CastResult_DebugDraw,
	TypedConstraint_btConstraintInfo2_getJ2angularAxis = CLIB.btTypedConstraint_btConstraintInfo2_getJ2angularAxis,
	TypedConstraint_btConstraintInfo2_getJ1angularAxis = CLIB.btTypedConstraint_btConstraintInfo2_getJ1angularAxis,
	TypedConstraint_btConstraintInfo2_getFps = CLIB.btTypedConstraint_btConstraintInfo2_getFps,
	GImpactBvh_new2 = CLIB.btGImpactBvh_new2,
	SoftBody_Config_setKKHR = CLIB.btSoftBody_Config_setKKHR,
	SoftBody_Element_getTag = CLIB.btSoftBody_Element_getTag,
	CompoundShapeChild_setNode = CLIB.btCompoundShapeChild_setNode,
	ManifoldResult_calculateCombinedContactStiffness = CLIB.btManifoldResult_calculateCombinedContactStiffness,
	DbvtBroadphase_getDeferedcollide = CLIB.btDbvtBroadphase_getDeferedcollide,
	TranslationalLimitMotor2_getSpringDampingLimited = CLIB.btTranslationalLimitMotor2_getSpringDampingLimited,
	QuantizedBvh_deSerializeFloat = CLIB.btQuantizedBvh_deSerializeFloat,
	TypedConstraint_btConstraintInfo2_new = CLIB.btTypedConstraint_btConstraintInfo2_new,
	CollisionAlgorithmConstructionInfo_getManifold = CLIB.btCollisionAlgorithmConstructionInfo_getManifold,
	TypedConstraint_btConstraintInfo1_setNumConstraintRows = CLIB.btTypedConstraint_btConstraintInfo1_setNumConstraintRows,
	TypedConstraint_btConstraintInfo1_setNub = CLIB.btTypedConstraint_btConstraintInfo1_setNub,
	MultiBody_worldPosToLocal = CLIB.btMultiBody_worldPosToLocal,
	TetrahedronShapeEx_new = CLIB.btTetrahedronShapeEx_new,
	TypedConstraint_btConstraintInfo1_getNumConstraintRows = CLIB.btTypedConstraint_btConstraintInfo1_getNumConstraintRows,
	TypedConstraint_btConstraintInfo1_new = CLIB.btTypedConstraint_btConstraintInfo1_new,
	CapsuleShapeX_new = CLIB.btCapsuleShapeX_new,
	DefaultMotionState_setUserPointer = CLIB.btDefaultMotionState_setUserPointer,
	ConvexTriangleCallback_new = CLIB.btConvexTriangleCallback_new,
	MultiBodySolverConstraint_setOrgDofIndex = CLIB.btMultiBodySolverConstraint_setOrgDofIndex,
	RigidBody_getAngularFactor = CLIB.btRigidBody_getAngularFactor,
	JointFeedback_setAppliedTorqueBodyA = CLIB.btJointFeedback_setAppliedTorqueBodyA,
	Dbvt_sStkNN_setB = CLIB.btDbvt_sStkNN_setB,
	RigidBody_btRigidBodyConstructionInfo_getAdditionalDamping = CLIB.btRigidBody_btRigidBodyConstructionInfo_getAdditionalDamping,
	Point2PointConstraint_getInfo1NonVirtual = CLIB.btPoint2PointConstraint_getInfo1NonVirtual,
	CollisionWorld_LocalRayResult_new = CLIB.btCollisionWorld_LocalRayResult_new,
	JointFeedback_getAppliedForceBodyA = CLIB.btJointFeedback_getAppliedForceBodyA,
	JointFeedback_new = CLIB.btJointFeedback_new,
	TriangleShapeEx_overlap_test_conservative = CLIB.btTriangleShapeEx_overlap_test_conservative,
	TriangleShapeEx_applyTransform = CLIB.btTriangleShapeEx_applyTransform,
	TriangleShapeEx_new3 = CLIB.btTriangleShapeEx_new3,
	ManifoldPoint_setContactMotion1 = CLIB.btManifoldPoint_setContactMotion1,
	SoftBody_sCti_getNormal = CLIB.btSoftBody_sCti_getNormal,
	ConvexTriangleCallback_clearCache = CLIB.btConvexTriangleCallback_clearCache,
	ContactSolverInfoData_setGlobalCfm = CLIB.btContactSolverInfoData_setGlobalCfm,
	CollisionObject_setCcdSweptSphereRadius = CLIB.btCollisionObject_setCcdSweptSphereRadius,
	TriangleShapeEx_new = CLIB.btTriangleShapeEx_new,
	PrimitiveTriangle_delete = CLIB.btPrimitiveTriangle_delete,
	CollisionWorld_AllHitsRayResultCallback_getHitFractions = CLIB.btCollisionWorld_AllHitsRayResultCallback_getHitFractions,
	PrimitiveTriangle_setPlane = CLIB.btPrimitiveTriangle_setPlane,
	PrimitiveTriangle_setMargin = CLIB.btPrimitiveTriangle_setMargin,
	PrimitiveTriangle_setDummy = CLIB.btPrimitiveTriangle_setDummy,
	PersistentManifold_replaceContactPoint = CLIB.btPersistentManifold_replaceContactPoint,
	TranslationalLimitMotor2_setCurrentLimitErrorHi = CLIB.btTranslationalLimitMotor2_setCurrentLimitErrorHi,
	PrimitiveTriangle_overlap_test_conservative = CLIB.btPrimitiveTriangle_overlap_test_conservative,
	PrimitiveTriangle_getVertices = CLIB.btPrimitiveTriangle_getVertices,
	PrimitiveTriangle_getPlane = CLIB.btPrimitiveTriangle_getPlane,
	CollisionWorld_ClosestRayResultCallback_new = CLIB.btCollisionWorld_ClosestRayResultCallback_new,
	MinkowskiSumShape_setTransformB = CLIB.btMinkowskiSumShape_setTransformB,
	SoftBody_Body_getSoft = CLIB.btSoftBody_Body_getSoft,
	GImpactBvh_buildSet = CLIB.btGImpactBvh_buildSet,
	Dbvt_sStkNPS_new = CLIB.btDbvt_sStkNPS_new,
	PrimitiveTriangle_getMargin = CLIB.btPrimitiveTriangle_getMargin,
	SoftBody_getVolume = CLIB.btSoftBody_getVolume,
	MultiBody_setUseGyroTerm = CLIB.btMultiBody_setUseGyroTerm,
	Dispatcher_getManifoldByIndexInternal = CLIB.btDispatcher_getManifoldByIndexInternal,
	IndexedMesh_getNumVertices = CLIB.btIndexedMesh_getNumVertices,
	PrimitiveTriangle_clip_triangle = CLIB.btPrimitiveTriangle_clip_triangle,
	SimulationIslandManager_new = CLIB.btSimulationIslandManager_new,
	CollisionDispatcher_registerCollisionCreateFunc = CLIB.btCollisionDispatcher_registerCollisionCreateFunc,
	SoftBody_Config_getKPR = CLIB.btSoftBody_Config_getKPR,
	PointCollector_getPointInWorld = CLIB.btPointCollector_getPointInWorld,
	DbvtNodePtr_array_at = CLIB.btDbvtNodePtr_array_at,
	HingeConstraint_setAxis = CLIB.btHingeConstraint_setAxis,
	CollisionObjectWrapper_setShape = CLIB.btCollisionObjectWrapper_setShape,
	CollisionConfiguration_getCollisionAlgorithmCreateFunc = CLIB.btCollisionConfiguration_getCollisionAlgorithmCreateFunc,
	Point2PointConstraint_getInfo2NonVirtual = CLIB.btPoint2PointConstraint_getInfo2NonVirtual,
	BU_Simplex1to4_new5 = CLIB.btBU_Simplex1to4_new5,
	CollisionObject_setContactProcessingThreshold = CLIB.btCollisionObject_setContactProcessingThreshold,
	SoftBody_checkLink = CLIB.btSoftBody_checkLink,
	OptimizedBvhNode_setTriangleIndex = CLIB.btOptimizedBvhNode_setTriangleIndex,
	RigidBody_btRigidBodyConstructionInfo_getAdditionalDampingFactor = CLIB.btRigidBody_btRigidBodyConstructionInfo_getAdditionalDampingFactor,
	CollisionWorld_LocalConvexResult_setHitNormalLocal = CLIB.btCollisionWorld_LocalConvexResult_setHitNormalLocal,
	Generic6DofConstraint_calculateTransforms2 = CLIB.btGeneric6DofConstraint_calculateTransforms2,
	RigidBody_getFlags = CLIB.btRigidBody_getFlags,
	TriangleShape_getVertices1 = CLIB.btTriangleShape_getVertices1,
	TriangleShape_getVertexPtr = CLIB.btTriangleShape_getVertexPtr,
	TriangleShape_getPlaneEquation = CLIB.btTriangleShape_getPlaneEquation,
	TriangleShape_calcNormal = CLIB.btTriangleShape_calcNormal,
	RigidBody_getLinearDamping = CLIB.btRigidBody_getLinearDamping,
	TriangleShape_new2 = CLIB.btTriangleShape_new2,
	MultiBodyConstraintSolver_new = CLIB.btMultiBodyConstraintSolver_new,
	SoftBody_Link_getRl = CLIB.btSoftBody_Link_getRl,
	Generic6DofConstraint_updateRHS = CLIB.btGeneric6DofConstraint_updateRHS,
	TriangleShape_new = CLIB.btTriangleShape_new,
	TriangleMeshShape_localGetSupportingVertexWithoutMargin = CLIB.btTriangleMeshShape_localGetSupportingVertexWithoutMargin,
	BvhTree_get_node_pointer = CLIB.btBvhTree_get_node_pointer,
	SoftBodyWorldInfo_setAir_density = CLIB.btSoftBodyWorldInfo_setAir_density,
	AlignedObjectArray_btSoftBody_Note_size = CLIB.btAlignedObjectArray_btSoftBody_Note_size,
	ConeTwistConstraint_GetPointForAngle = CLIB.btConeTwistConstraint_GetPointForAngle,
	ManifoldPoint_getPositionWorldOnB = CLIB.btManifoldPoint_getPositionWorldOnB,
	StridingMeshInterface_unLockVertexBase = CLIB.btStridingMeshInterface_unLockVertexBase,
	GImpactMeshShape_getMeshInterface = CLIB.btGImpactMeshShape_getMeshInterface,
	MaterialProperties_getTriangleMaterialStride = CLIB.btMaterialProperties_getTriangleMaterialStride,
	SoftBody_RayFromToCaster_getMint = CLIB.btSoftBody_RayFromToCaster_getMint,
	SoftBody_Face_setNormal = CLIB.btSoftBody_Face_setNormal,
	SoftBody_getTag = CLIB.btSoftBody_getTag,
	GImpactBvh_setPrimitiveManager = CLIB.btGImpactBvh_setPrimitiveManager,
	SoftBodyConcaveCollisionAlgorithm_CreateFunc_new = CLIB.btSoftBodyConcaveCollisionAlgorithm_CreateFunc_new,
	TriangleMesh_findOrAddVertex = CLIB.btTriangleMesh_findOrAddVertex,
	TriangleMesh_addTriangleIndices = CLIB.btTriangleMesh_addTriangleIndices,
	CollisionObject_checkCollideWith = CLIB.btCollisionObject_checkCollideWith,
	AlignedObjectArray_btSoftBodyPtr_resizeNoInitialize = CLIB.btAlignedObjectArray_btSoftBodyPtr_resizeNoInitialize,
	DiscreteCollisionDetectorInterface_Result_delete = CLIB.btDiscreteCollisionDetectorInterface_Result_delete,
	CollisionObject_getContactProcessingThreshold = CLIB.btCollisionObject_getContactProcessingThreshold,
	TriangleMesh_new = CLIB.btTriangleMesh_new,
	SoftBody_Tetra_getC1 = CLIB.btSoftBody_Tetra_getC1,
	SoftBody_Cluster_setFramexform = CLIB.btSoftBody_Cluster_setFramexform,
	RigidBody_translate = CLIB.btRigidBody_translate,
	Generic6DofSpring2Constraint_calculateTransforms = CLIB.btGeneric6DofSpring2Constraint_calculateTransforms,
	Dbvt_ICollide_Descent = CLIB.btDbvt_ICollide_Descent,
	NodeOverlapCallback_delete = CLIB.btNodeOverlapCallback_delete,
	MultiBody_getUserIndex2 = CLIB.btMultiBody_getUserIndex2,
	ManifoldPoint_new2 = CLIB.btManifoldPoint_new2,
	RigidBody_removeConstraintRef = CLIB.btRigidBody_removeConstraintRef,
	ContactSolverInfoData_delete = CLIB.btContactSolverInfoData_delete,
	SoftBody_RContact_getC0 = CLIB.btSoftBody_RContact_getC0,
	MultiBody_setNumLinks = CLIB.btMultiBody_setNumLinks,
	GImpactCompoundShape_new = CLIB.btGImpactCompoundShape_new,
	ContactSolverInfoData_getMinimumSolverBatchSize = CLIB.btContactSolverInfoData_getMinimumSolverBatchSize,
	SliderConstraint_getSoftnessLimAng = CLIB.btSliderConstraint_getSoftnessLimAng,
	BU_Simplex1to4_new3 = CLIB.btBU_Simplex1to4_new3,
	MultiBody_getRVector = CLIB.btMultiBody_getRVector,
	AngularLimit_isLimit = CLIB.btAngularLimit_isLimit,
	StridingMeshInterface_InternalProcessAllTriangles = CLIB.btStridingMeshInterface_InternalProcessAllTriangles,
	SoftBody_Body_getRigid = CLIB.btSoftBody_Body_getRigid,
	OverlappingPairCache_cleanProxyFromPairs = CLIB.btOverlappingPairCache_cleanProxyFromPairs,
	BvhTree_setNodeBound = CLIB.btBvhTree_setNodeBound,
	ConvexCast_CastResult_setFraction = CLIB.btConvexCast_CastResult_setFraction,
	ConvexCast_CastResult_getHitPoint = CLIB.btConvexCast_CastResult_getHitPoint,
	MultibodyLink_getAxisTop = CLIB.btMultibodyLink_getAxisTop,
	SliderConstraint_setMaxAngMotorForce = CLIB.btSliderConstraint_setMaxAngMotorForce,
	TriangleIndexVertexMaterialArray_addMaterialProperties = CLIB.btTriangleIndexVertexMaterialArray_addMaterialProperties,
	Point2PointConstraint_getUseSolveConstraintObsolete = CLIB.btPoint2PointConstraint_getUseSolveConstraintObsolete,
	TriangleIndexVertexMaterialArray_new = CLIB.btTriangleIndexVertexMaterialArray_new,
	MaterialProperties_delete = CLIB.btMaterialProperties_delete,
	Generic6DofConstraint_getCalculatedTransformB = CLIB.btGeneric6DofConstraint_getCalculatedTransformB,
	IDebugDrawWrapper_new = CLIB.btIDebugDrawWrapper_new,
	MaterialProperties_setTriangleType = CLIB.btMaterialProperties_setTriangleType,
	ConstraintSetting_new = CLIB.btConstraintSetting_new,
	MaterialProperties_setNumTriangles = CLIB.btMaterialProperties_setNumTriangles,
	OverlapFilterCallback_delete = CLIB.btOverlapFilterCallback_delete,
	MaterialProperties_setNumMaterials = CLIB.btMaterialProperties_setNumMaterials,
	BroadphaseProxy_isNonMoving = CLIB.btBroadphaseProxy_isNonMoving,
	UnionFind_getElement = CLIB.btUnionFind_getElement,
	MaterialProperties_setMaterialStride = CLIB.btMaterialProperties_setMaterialStride,
	Dbvt_sStkNN_setA = CLIB.btDbvt_sStkNN_setA,
	ConeTwistConstraint_setMaxMotorImpulse = CLIB.btConeTwistConstraint_setMaxMotorImpulse,
	SoftBody_applyClusters = CLIB.btSoftBody_applyClusters,
	CollisionObject_setUserPointer = CLIB.btCollisionObject_setUserPointer,
	MaterialProperties_setMaterialBase = CLIB.btMaterialProperties_setMaterialBase,
	MaterialProperties_getTriangleType = CLIB.btMaterialProperties_getTriangleType,
	RotationalLimitMotor2_setMaxMotorForce = CLIB.btRotationalLimitMotor2_setMaxMotorForce,
	OptimizedBvh_serializeInPlace = CLIB.btOptimizedBvh_serializeInPlace,
	SoftBody_Config_getDsequence = CLIB.btSoftBody_Config_getDsequence,
	MaterialProperties_getMaterialStride = CLIB.btMaterialProperties_getMaterialStride,
	AlignedObjectArray_btSoftBody_Link_set = CLIB.btAlignedObjectArray_btSoftBody_Link_set,
	MultiBodyDynamicsWorld_removeMultiBody = CLIB.btMultiBodyDynamicsWorld_removeMultiBody,
	MaterialProperties_new = CLIB.btMaterialProperties_new,
	TriangleIndexVertexArray_addIndexedMesh = CLIB.btTriangleIndexVertexArray_addIndexedMesh,
	CollisionObject_setUserIndex = CLIB.btCollisionObject_setUserIndex,
	MultiBody_getBaseInertia = CLIB.btMultiBody_getBaseInertia,
	IndexedMesh_setVertexStride = CLIB.btIndexedMesh_setVertexStride,
	MultiBody_getJointPosMultiDof = CLIB.btMultiBody_getJointPosMultiDof,
	Dispatcher_releaseManifold = CLIB.btDispatcher_releaseManifold,
	GImpactShapeInterface_needsRetrieveTetrahedrons = CLIB.btGImpactShapeInterface_needsRetrieveTetrahedrons,
	IndexedMesh_setTriangleIndexStride = CLIB.btIndexedMesh_setTriangleIndexStride,
	TranslationalLimitMotor_new = CLIB.btTranslationalLimitMotor_new,
	MultiBody_getLinkForce = CLIB.btMultiBody_getLinkForce,
	IndexedMesh_setTriangleIndexBase = CLIB.btIndexedMesh_setTriangleIndexBase,
	IndexedMesh_setNumVertices = CLIB.btIndexedMesh_setNumVertices,
	IndexedMesh_setNumTriangles = CLIB.btIndexedMesh_setNumTriangles,
	GjkPairDetector_getCachedSeparatingAxis = CLIB.btGjkPairDetector_getCachedSeparatingAxis,
	TranslationalLimitMotor2_setCurrentLimitError = CLIB.btTranslationalLimitMotor2_setCurrentLimitError,
	RigidBody_predictIntegratedTransform = CLIB.btRigidBody_predictIntegratedTransform,
	CollisionObject_setFriction = CLIB.btCollisionObject_setFriction,
	GImpactShapeInterface_getChildAabb = CLIB.btGImpactShapeInterface_getChildAabb,
	ConeTwistConstraint_setMotorTarget = CLIB.btConeTwistConstraint_setMotorTarget,
	TranslationalLimitMotor2_getEnableSpring = CLIB.btTranslationalLimitMotor2_getEnableSpring,
	HingeConstraint_hasLimit = CLIB.btHingeConstraint_hasLimit,
	Generic6DofConstraint_calcAnchorPos = CLIB.btGeneric6DofConstraint_calcAnchorPos,
	GImpactBvh_getNodeCount = CLIB.btGImpactBvh_getNodeCount,
	ConstraintSetting_getImpulseClamp = CLIB.btConstraintSetting_getImpulseClamp,
	MultiBodySolverConstraint_setLowerLimit = CLIB.btMultiBodySolverConstraint_setLowerLimit,
	IndexedMesh_getVertexBase = CLIB.btIndexedMesh_getVertexBase,
	IndexedMesh_getTriangleIndexStride = CLIB.btIndexedMesh_getTriangleIndexStride,
	GImpactShapeInterface_hasBoxSet = CLIB.btGImpactShapeInterface_hasBoxSet,
	Dispatcher_getNewManifold = CLIB.btDispatcher_getNewManifold,
	ContactSolverInfoData_getSor = CLIB.btContactSolverInfoData_getSor,
	IndexedMesh_getTriangleIndexBase = CLIB.btIndexedMesh_getTriangleIndexBase,
	PrimitiveTriangle_get_edge_plane = CLIB.btPrimitiveTriangle_get_edge_plane,
	IndexedMesh_getNumTriangles = CLIB.btIndexedMesh_getNumTriangles,
	CollisionWorld_debugDrawObject = CLIB.btCollisionWorld_debugDrawObject,
	IndexedMesh_new = CLIB.btIndexedMesh_new,
	DbvtBroadphase_setReleasepaircache = CLIB.btDbvtBroadphase_setReleasepaircache,
	Generic6DofSpring2Constraint_matrixToEulerXZY = CLIB.btGeneric6DofSpring2Constraint_matrixToEulerXZY,
	InternalTriangleIndexCallbackWrapper_new = CLIB.btInternalTriangleIndexCallbackWrapper_new,
	TriangleCallback_delete = CLIB.btTriangleCallback_delete,
	RigidBody_applyCentralForce = CLIB.btRigidBody_applyCentralForce,
	CollisionWorld_AllHitsRayResultCallback_getRayToWorld = CLIB.btCollisionWorld_AllHitsRayResultCallback_getRayToWorld,
	TriangleBuffer_getTriangle = CLIB.btTriangleBuffer_getTriangle,
	HashedOverlappingPairCache_needsBroadphaseCollision = CLIB.btHashedOverlappingPairCache_needsBroadphaseCollision,
	GhostObject_rayTest = CLIB.btGhostObject_rayTest,
	HeightfieldTerrainShape_setUseDiamondSubdivision = CLIB.btHeightfieldTerrainShape_setUseDiamondSubdivision,
	TriangleBuffer_getNumTriangles = CLIB.btTriangleBuffer_getNumTriangles,
	QuantizedBvhTree_isLeafNode = CLIB.btQuantizedBvhTree_isLeafNode,
	TranslationalLimitMotor_setDamping = CLIB.btTranslationalLimitMotor_setDamping,
	Triangle_delete = CLIB.btTriangle_delete,
	Triangle_setVertex2 = CLIB.btTriangle_setVertex2,
	PersistentManifold_delete = CLIB.btPersistentManifold_delete,
	Triangle_setVertex1 = CLIB.btTriangle_setVertex1,
	Triangle_setVertex0 = CLIB.btTriangle_setVertex0,
	Triangle_setTriangleIndex = CLIB.btTriangle_setTriangleIndex,
	MultiBody_delete = CLIB.btMultiBody_delete,
	SoftBody_getTetras = CLIB.btSoftBody_getTetras,
	HeightfieldTerrainShape_setUseZigzagSubdivision = CLIB.btHeightfieldTerrainShape_setUseZigzagSubdivision,
	CollisionAlgorithmCreateFunc_delete = CLIB.btCollisionAlgorithmCreateFunc_delete,
	SoftBody_Config_getKDF = CLIB.btSoftBody_Config_getKDF,
	Triangle_setPartId = CLIB.btTriangle_setPartId,
	SliderConstraint_setDampingDirLin = CLIB.btSliderConstraint_setDampingDirLin,
	CollisionWorld_RayResultCallbackWrapper_new = CLIB.btCollisionWorld_RayResultCallbackWrapper_new,
	SliderConstraint_setUpperLinLimit = CLIB.btSliderConstraint_setUpperLinLimit,
	Triangle_getPartId = CLIB.btTriangle_getPartId,
	RigidBody_computeAngularImpulseDenominator = CLIB.btRigidBody_computeAngularImpulseDenominator,
	ContactSolverInfoData_getFrictionCfm = CLIB.btContactSolverInfoData_getFrictionCfm,
	SoftBody_Impulse_getDrift = CLIB.btSoftBody_Impulse_getDrift,
	GhostPairCallback_new = CLIB.btGhostPairCallback_new,
	Generic6DofSpring2Constraint_setLinearUpperLimit = CLIB.btGeneric6DofSpring2Constraint_setLinearUpperLimit,
	ConeTwistConstraint_getBiasFactor = CLIB.btConeTwistConstraint_getBiasFactor,
	SoftBody_Body_activate = CLIB.btSoftBody_Body_activate,
	BroadphaseProxy_getCollisionFilterGroup = CLIB.btBroadphaseProxy_getCollisionFilterGroup,
	ManifoldPoint_setContactMotion2 = CLIB.btManifoldPoint_setContactMotion2,
	ConvexSeparatingDistanceUtil_updateSeparatingDistance = CLIB.btConvexSeparatingDistanceUtil_updateSeparatingDistance,
	ConvexSeparatingDistanceUtil_initSeparatingDistance = CLIB.btConvexSeparatingDistanceUtil_initSeparatingDistance,
	ConvexSeparatingDistanceUtil_getConservativeSeparatingDistance = CLIB.btConvexSeparatingDistanceUtil_getConservativeSeparatingDistance,
	MultiBodySliderConstraint_getFrameInB = CLIB.btMultiBodySliderConstraint_getFrameInB,
	TranslationalLimitMotor_new2 = CLIB.btTranslationalLimitMotor_new2,
	RigidBody_setGravity = CLIB.btRigidBody_setGravity,
	AlignedObjectArray_btIndexedMesh_resizeNoInitialize = CLIB.btAlignedObjectArray_btIndexedMesh_resizeNoInitialize,
	DbvtAabbMm_Expand = CLIB.btDbvtAabbMm_Expand,
	SoftBody_Body_invWorldInertia = CLIB.btSoftBody_Body_invWorldInertia,
	SoftBody_Joint_Terminate = CLIB.btSoftBody_Joint_Terminate,
	SoftSoftCollisionAlgorithm_new2 = CLIB.btSoftSoftCollisionAlgorithm_new2,
	TransformUtil_calculateVelocityQuaternion = CLIB.btTransformUtil_calculateVelocityQuaternion,
	AlignedObjectArray_btSoftBody_Note_at = CLIB.btAlignedObjectArray_btSoftBody_Note_at,
	Dbvt_optimizeBottomUp = CLIB.btDbvt_optimizeBottomUp,
	StridingMeshInterface_setPremadeAabb = CLIB.btStridingMeshInterface_setPremadeAabb,
	SoftBody_Body_applyVAImpulse = CLIB.btSoftBody_Body_applyVAImpulse,
	SoftBody_Cluster_getLocii = CLIB.btSoftBody_Cluster_getLocii,
	BU_Simplex1to4_reset = CLIB.btBU_Simplex1to4_reset,
	SoftBody_Pose_getCom = CLIB.btSoftBody_Pose_getCom,
	SliderConstraint_getAncorInB = CLIB.btSliderConstraint_getAncorInB,
	GImpactQuantizedBvh_getRightNode = CLIB.btGImpactQuantizedBvh_getRightNode,
	BU_Simplex1to4_new2 = CLIB.btBU_Simplex1to4_new2,
	StridingMeshInterface_delete = CLIB.btStridingMeshInterface_delete,
	StridingMeshInterface_setScaling = CLIB.btStridingMeshInterface_setScaling,
	MultibodyLink_setJointDamping = CLIB.btMultibodyLink_setJointDamping,
	SoftBody_addVelocity2 = CLIB.btSoftBody_addVelocity2,
	TransformUtil_calculateDiffAxisAngleQuaternion = CLIB.btTransformUtil_calculateDiffAxisAngleQuaternion,
	StridingMeshInterface_serialize = CLIB.btStridingMeshInterface_serialize,
	ConvexHullShape_new4 = CLIB.btConvexHullShape_new4,
	StridingMeshInterface_preallocateVertices = CLIB.btStridingMeshInterface_preallocateVertices,
	SoftBody_Element_delete = CLIB.btSoftBody_Element_delete,
	UnionFind_find2 = CLIB.btUnionFind_find2,
	PersistentManifold_getCacheEntry = CLIB.btPersistentManifold_getCacheEntry,
	StridingMeshInterface_hasPremadeAabb = CLIB.btStridingMeshInterface_hasPremadeAabb,
	GImpactMeshShape_getMeshPartCount = CLIB.btGImpactMeshShape_getMeshPartCount,
	StridingMeshInterface_getScaling = CLIB.btStridingMeshInterface_getScaling,
	TypedConstraint_btConstraintInfo2_getNumIterations = CLIB.btTypedConstraint_btConstraintInfo2_getNumIterations,
	StridingMeshInterface_getLockedVertexIndexBase = CLIB.btStridingMeshInterface_getLockedVertexIndexBase,
	PersistentManifold_getCompanionIdB = CLIB.btPersistentManifold_getCompanionIdB,
	DiscreteDynamicsWorld_debugDrawConstraint = CLIB.btDiscreteDynamicsWorld_debugDrawConstraint,
	StaticPlaneShape_getPlaneNormal = CLIB.btStaticPlaneShape_getPlaneNormal,
	StaticPlaneShape_getPlaneConstant = CLIB.btStaticPlaneShape_getPlaneConstant,
	SliderConstraint_getRestitutionDirLin = CLIB.btSliderConstraint_getRestitutionDirLin,
	SoftBody_Pose_getBvolume = CLIB.btSoftBody_Pose_getBvolume,
	SoftBody_Node_getBattach = CLIB.btSoftBody_Node_getBattach,
	DefaultCollisionConstructionInfo_setPersistentManifoldPool = CLIB.btDefaultCollisionConstructionInfo_setPersistentManifoldPool,
	SphereTriangleCollisionAlgorithm_new2 = CLIB.btSphereTriangleCollisionAlgorithm_new2,
	SoftBody_appendNote2 = CLIB.btSoftBody_appendNote2,
	SoftBody_sCti_getOffset = CLIB.btSoftBody_sCti_getOffset,
	BroadphaseInterface_getOverlappingPairCache = CLIB.btBroadphaseInterface_getOverlappingPairCache,
	CollisionWorld_updateAabbs = CLIB.btCollisionWorld_updateAabbs,
	SphereSphereCollisionAlgorithm_CreateFunc_new = CLIB.btSphereSphereCollisionAlgorithm_CreateFunc_new,
	PrimitiveManagerBase_get_primitive_count = CLIB.btPrimitiveManagerBase_get_primitive_count,
	CollisionWorld_getNumCollisionObjects = CLIB.btCollisionWorld_getNumCollisionObjects,
	SoftBody_generateBendingConstraints = CLIB.btSoftBody_generateBendingConstraints,
	HingeConstraint_setLimit4 = CLIB.btHingeConstraint_setLimit4,
	ConvexPolyhedron_setExtents = CLIB.btConvexPolyhedron_setExtents,
	ConvexTriangleCallback_getAabbMin = CLIB.btConvexTriangleCallback_getAabbMin,
	DbvtProxy_setLeaf = CLIB.btDbvtProxy_setLeaf,
	SphereBoxCollisionAlgorithm_getSpherePenetration = CLIB.btSphereBoxCollisionAlgorithm_getSpherePenetration,
	SoftBody_Joint_Specs_delete = CLIB.btSoftBody_Joint_Specs_delete,
	SoftBody_Joint_getSplit = CLIB.btSoftBody_Joint_getSplit,
	SoftBody_appendTetra = CLIB.btSoftBody_appendTetra,
	SphereBoxCollisionAlgorithm_new = CLIB.btSphereBoxCollisionAlgorithm_new,
	MinkowskiSumShape_setTransformA = CLIB.btMinkowskiSumShape_setTransformA,
	SphereBoxCollisionAlgorithm_CreateFunc_new = CLIB.btSphereBoxCollisionAlgorithm_CreateFunc_new,
	MultibodyLink_getUserPtr = CLIB.btMultibodyLink_getUserPtr,
	HingeConstraint_getAFrame = CLIB.btHingeConstraint_getAFrame,
	SparseSdf3_Reset = CLIB.btSparseSdf3_Reset,
	SoftBody_Config_getCollisions = CLIB.btSoftBody_Config_getCollisions,
	ConvexInternalShape_getImplicitShapeDimensions = CLIB.btConvexInternalShape_getImplicitShapeDimensions,
	SoftBody_initializeClusters = CLIB.btSoftBody_initializeClusters,
	SparseSdf3_RemoveReferences = CLIB.btSparseSdf3_RemoveReferences,
	ContactSolverInfoData_setMaxGyroscopicForce = CLIB.btContactSolverInfoData_setMaxGyroscopicForce,
	Dbvt_sStkNPS_setNode = CLIB.btDbvt_sStkNPS_setNode,
	SoftSoftCollisionAlgorithm_new = CLIB.btSoftSoftCollisionAlgorithm_new,
	SoftSoftCollisionAlgorithm_CreateFunc_new = CLIB.btSoftSoftCollisionAlgorithm_CreateFunc_new,
	SoftRigidDynamicsWorld_setDrawFlags = CLIB.btSoftRigidDynamicsWorld_setDrawFlags,
	SoftBody_Node_getArea = CLIB.btSoftBody_Node_getArea,
	MultibodyLink_getCachedRotParentToThis = CLIB.btMultibodyLink_getCachedRotParentToThis,
	ConeTwistConstraint_isMaxMotorImpulseNormalized = CLIB.btConeTwistConstraint_isMaxMotorImpulseNormalized,
	AlignedObjectArray_btSoftBody_MaterialPtr_resizeNoInitialize = CLIB.btAlignedObjectArray_btSoftBody_MaterialPtr_resizeNoInitialize,
	HingeConstraint_getMotorTargetVelocity = CLIB.btHingeConstraint_getMotorTargetVelocity,
	SoftRigidCollisionAlgorithm_new = CLIB.btSoftRigidCollisionAlgorithm_new,
	CollisionWorld_removeCollisionObject = CLIB.btCollisionWorld_removeCollisionObject,
	SoftRigidDynamicsWorld_getDrawFlags = CLIB.btSoftRigidDynamicsWorld_getDrawFlags,
	HingeConstraint_getUseReferenceFrameA = CLIB.btHingeConstraint_getUseReferenceFrameA,
	PairSet_new = CLIB.btPairSet_new,
	MultibodyLink_setAppliedTorque = CLIB.btMultibodyLink_setAppliedTorque,
	SoftRigidDynamicsWorld_addSoftBody2 = CLIB.btSoftRigidDynamicsWorld_addSoftBody2,
	RigidBody_applyCentralImpulse = CLIB.btRigidBody_applyCentralImpulse,
	Generic6DofSpring2Constraint_setServoTarget = CLIB.btGeneric6DofSpring2Constraint_setServoTarget,
	SoftRigidDynamicsWorld_getSoftBodyArray = CLIB.btSoftRigidDynamicsWorld_getSoftBodyArray,
	ManifoldPoint_getContactMotion1 = CLIB.btManifoldPoint_getContactMotion1,
	CollisionWorld_ClosestRayResultCallback_setRayFromWorld = CLIB.btCollisionWorld_ClosestRayResultCallback_setRayFromWorld,
	GImpactShapeInterface_getBulletTriangle = CLIB.btGImpactShapeInterface_getBulletTriangle,
	SoftBodySolverOutput_delete = CLIB.btSoftBodySolverOutput_delete,
	SoftBodySolver_setNumberOfVelocityIterations = CLIB.btSoftBodySolver_setNumberOfVelocityIterations,
	Generic6DofSpring2Constraint_setServo = CLIB.btGeneric6DofSpring2Constraint_setServo,
	SoftBodySolver_setNumberOfPositionIterations = CLIB.btSoftBodySolver_setNumberOfPositionIterations,
	ConeShape_setConeUpIndex = CLIB.btConeShape_setConeUpIndex,
	SoftBodySolver_predictMotion = CLIB.btSoftBodySolver_predictMotion,
	CollisionWorld_AllHitsRayResultCallback_getHitNormalWorld = CLIB.btCollisionWorld_AllHitsRayResultCallback_getHitNormalWorld,
	SoftBodySolver_getNumberOfVelocityIterations = CLIB.btSoftBodySolver_getNumberOfVelocityIterations,
	Generic6DofConstraint_getFrameOffsetB = CLIB.btGeneric6DofConstraint_getFrameOffsetB,
	RigidBody_btRigidBodyConstructionInfo_getLocalInertia = CLIB.btRigidBody_btRigidBodyConstructionInfo_getLocalInertia,
	CollisionObject_getCollisionShape = CLIB.btCollisionObject_getCollisionShape,
	ConeTwistConstraint_getAFrame = CLIB.btConeTwistConstraint_getAFrame,
	SoftBodySolver_checkInitialized = CLIB.btSoftBodySolver_checkInitialized,
	CollisionWorld_RayResultCallbackWrapper_needsCollision = CLIB.btCollisionWorld_RayResultCallbackWrapper_needsCollision,
	RotationalLimitMotor_testLimitValue = CLIB.btRotationalLimitMotor_testLimitValue,
	ContactSolverInfoData_setFrictionErp = CLIB.btContactSolverInfoData_setFrictionErp,
	SoftBodyHelpers_DrawNodeTree = CLIB.btSoftBodyHelpers_DrawNodeTree,
	SoftBodyHelpers_DrawInfos = CLIB.btSoftBodyHelpers_DrawInfos,
	GjkPairDetector_getLastUsedMethod = CLIB.btGjkPairDetector_getLastUsedMethod,
	MultiBody_getParent = CLIB.btMultiBody_getParent,
	Triangle_getVertex1 = CLIB.btTriangle_getVertex1,
	DiscreteDynamicsWorld_setApplySpeculativeContactRestitution = CLIB.btDiscreteDynamicsWorld_setApplySpeculativeContactRestitution,
	RotationalLimitMotor_getMaxMotorForce = CLIB.btRotationalLimitMotor_getMaxMotorForce,
	SoftBodyHelpers_DrawClusterTree = CLIB.btSoftBodyHelpers_DrawClusterTree,
	SoftBodyHelpers_CreatePatchUV = CLIB.btSoftBodyHelpers_CreatePatchUV,
	AxisSweep3_setOverlappingPairUserCallback = CLIB.btAxisSweep3_setOverlappingPairUserCallback,
	SoftBody_RContact_getC1 = CLIB.btSoftBody_RContact_getC1,
	SoftBodyConcaveCollisionAlgorithm_new = CLIB.btSoftBodyConcaveCollisionAlgorithm_new,
	SoftBody_getTetraVertexNormalData = CLIB.btSoftBody_getTetraVertexNormalData,
	PolyhedralConvexShape_getEdge = CLIB.btPolyhedralConvexShape_getEdge,
	Generic6DofSpringConstraint_getStiffness = CLIB.btGeneric6DofSpringConstraint_getStiffness,
	CollisionShape_delete = CLIB.btCollisionShape_delete,
	Dbvt_array_at = CLIB.btDbvt_array_at,
	CollisionShape_getLocalScaling = CLIB.btCollisionShape_getLocalScaling,
	SoftBody_ImplicitFn_delete = CLIB.btSoftBody_ImplicitFn_delete,
	GImpactMeshShapePart_TrimeshPrimitiveManager_setPart = CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_setPart,
	MultibodyLink_setAppliedConstraintTorque = CLIB.btMultibodyLink_setAppliedConstraintTorque,
	MultiBodySliderConstraint_new2 = CLIB.btMultiBodySliderConstraint_new2,
	SoftBody_updateNormals = CLIB.btSoftBody_updateNormals,
	SoftBody_Config_setKLF = CLIB.btSoftBody_Config_setKLF,
	OptimizedBvh_deSerializeInPlace = CLIB.btOptimizedBvh_deSerializeInPlace,
	UniversalConstraint_getAnchor = CLIB.btUniversalConstraint_getAnchor,
	TranslationalLimitMotor2_setStopERP = CLIB.btTranslationalLimitMotor2_setStopERP,
	SoftBody_updateBounds = CLIB.btSoftBody_updateBounds,
	ConeTwistConstraint_new = CLIB.btConeTwistConstraint_new,
	CollisionWorld_rayTestSingle = CLIB.btCollisionWorld_rayTestSingle,
	SoftBody_Config_getKSRHR_CL = CLIB.btSoftBody_Config_getKSRHR_CL,
	SoftBody_translate = CLIB.btSoftBody_translate,
	SoftBody_transform = CLIB.btSoftBody_transform,
	CollisionWorld_getDispatcher = CLIB.btCollisionWorld_getDispatcher,
	CollisionObject_setSpinningFriction = CLIB.btCollisionObject_setSpinningFriction,
	SoftBody_Joint_Specs_setErp = CLIB.btSoftBody_Joint_Specs_setErp,
	SoftBody_solveConstraints = CLIB.btSoftBody_solveConstraints,
	SoftBody_solveCommonConstraints = CLIB.btSoftBody_solveCommonConstraints,
	SoftBody_solveClusters2 = CLIB.btSoftBody_solveClusters2,
	VoronoiSimplexSolver_closestPtPointTriangle = CLIB.btVoronoiSimplexSolver_closestPtPointTriangle,
	AlignedObjectArray_btPersistentManifoldPtr_new = CLIB.btAlignedObjectArray_btPersistentManifoldPtr_new,
	SoftBody_setVolumeMass = CLIB.btSoftBody_setVolumeMass,
	HingeConstraint_updateRHS = CLIB.btHingeConstraint_updateRHS,
	TranslationalLimitMotor2_getEnableMotor = CLIB.btTranslationalLimitMotor2_getEnableMotor,
	SoftBody_PSolve_Anchors = CLIB.btSoftBody_PSolve_Anchors,
	MultiBodyFixedConstraint_new = CLIB.btMultiBodyFixedConstraint_new,
	SoftBody_setWindVelocity = CLIB.btSoftBody_setWindVelocity,
	ConvexPointCloudShape_getUnscaledPoints = CLIB.btConvexPointCloudShape_getUnscaledPoints,
	SoftBody_setVelocity = CLIB.btSoftBody_setVelocity,
	SoftBody_setTotalMass = CLIB.btSoftBody_setTotalMass,
	SoftBody_setTag = CLIB.btSoftBody_setTag,
	Generic6DofConstraint_getCalculatedTransformA = CLIB.btGeneric6DofConstraint_getCalculatedTransformA,
	SoftBody_setSolver = CLIB.btSoftBody_setSolver,
	SoftBody_setRestLengthScale = CLIB.btSoftBody_setRestLengthScale,
	SoftBody_setPose = CLIB.btSoftBody_setPose,
	SoftBody_setMass = CLIB.btSoftBody_setMass,
	MultibodyLink_setEVector = CLIB.btMultibodyLink_setEVector,
	SoftBody_scale = CLIB.btSoftBody_scale,
	SoftBody_rotate = CLIB.btSoftBody_rotate,
	MultiBody_getBaseVel = CLIB.btMultiBody_getBaseVel,
	SoftBody_resetLinkRestLengths = CLIB.btSoftBody_resetLinkRestLengths,
	SoftBody_releaseClusters = CLIB.btSoftBody_releaseClusters,
	OverlapCallback_processOverlap = CLIB.btOverlapCallback_processOverlap,
	OptimizedBvh_new = CLIB.btOptimizedBvh_new,
	SoftBody_releaseCluster = CLIB.btSoftBody_releaseCluster,
	SoftBody_refine = CLIB.btSoftBody_refine,
	VoronoiSimplexSolver_getLastW = CLIB.btVoronoiSimplexSolver_getLastW,
	SoftBody_rayTest = CLIB.btSoftBody_rayTest,
	SoftBody_PSolve_SContacts = CLIB.btSoftBody_PSolve_SContacts,
	GImpactMeshShape_new = CLIB.btGImpactMeshShape_new,
	SoftBody_PSolve_RContacts = CLIB.btSoftBody_PSolve_RContacts,
	TranslationalLimitMotor2_setBounce = CLIB.btTranslationalLimitMotor2_setBounce,
	SoftBody_RayFromToCaster_setRayFrom = CLIB.btSoftBody_RayFromToCaster_setRayFrom,
	SoftBody_prepareClusters = CLIB.btSoftBody_prepareClusters,
	SoftBody_LJoint_getRpos = CLIB.btSoftBody_LJoint_getRpos,
	SoftBody_pointersToIndices = CLIB.btSoftBody_pointersToIndices,
	Dbvt_empty = CLIB.btDbvt_empty,
	MultibodyLink_setJointFeedback = CLIB.btMultibodyLink_setJointFeedback,
	Dbvt_sStkNN_getA = CLIB.btDbvt_sStkNN_getA,
	SoftBody_integrateMotion = CLIB.btSoftBody_integrateMotion,
	BvhTree_getRightNode = CLIB.btBvhTree_getRightNode,
	MultiBody_getBaseOmega = CLIB.btMultiBody_getBaseOmega,
	MultibodyLink_getAppliedTorque = CLIB.btMultibodyLink_getAppliedTorque,
	ContactSolverInfoData_getFriction = CLIB.btContactSolverInfoData_getFriction,
	SoftBody_initDefaults = CLIB.btSoftBody_initDefaults,
	MultiBody_addLinkTorque = CLIB.btMultiBody_addLinkTorque,
	SoftBody_indicesToPointers = CLIB.btSoftBody_indicesToPointers,
	PrimitiveTriangle_getDummy = CLIB.btPrimitiveTriangle_getDummy,
	RigidBody_computeImpulseDenominator = CLIB.btRigidBody_computeImpulseDenominator,
	SoftBodyWorldInfo_setGravity = CLIB.btSoftBodyWorldInfo_setGravity,
	RotationalLimitMotor2_setMotorERP = CLIB.btRotationalLimitMotor2_setMotorERP,
	BvhTriangleMeshShape_getOwnsBvh = CLIB.btBvhTriangleMeshShape_getOwnsBvh,
	SoftBody_Joint_Specs_new = CLIB.btSoftBody_Joint_Specs_new,
	SoftBody_getUserIndexMapping = CLIB.btSoftBody_getUserIndexMapping,
	SoftBody_getTimeacc = CLIB.btSoftBody_getTimeacc,
	TriangleMesh_getNumTriangles = CLIB.btTriangleMesh_getNumTriangles,
	SoftBody_appendNote5 = CLIB.btSoftBody_appendNote5,
	SoftBody_getSoftBodySolver = CLIB.btSoftBody_getSoftBodySolver,
	SoftBody_getScontacts = CLIB.btSoftBody_getScontacts,
	DiscreteCollisionDetectorInterface_ClosestPointInput_setMaximumDistanceSquared = CLIB.btDiscreteCollisionDetectorInterface_ClosestPointInput_setMaximumDistanceSquared,
	SoftBody_getRestLengthScale = CLIB.btSoftBody_getRestLengthScale,
	SoftBody_getRcontacts = CLIB.btSoftBody_getRcontacts,
	MultiBodySolverConstraint_getAppliedPushImpulse = CLIB.btMultiBodySolverConstraint_getAppliedPushImpulse,
	SoftBody_getPose = CLIB.btSoftBody_getPose,
	AxisSweep3_quantize = CLIB.btAxisSweep3_quantize,
	SoftBody_getNdbvt = CLIB.btSoftBody_getNdbvt,
	SoftBody_getMaterials = CLIB.btSoftBody_getMaterials,
	SoftBody_getMass = CLIB.btSoftBody_getMass,
	SoftBody_getJoints = CLIB.btSoftBody_getJoints,
	SoftBody_getInitialWorldTransform = CLIB.btSoftBody_getInitialWorldTransform,
	SoftBody_getFdbvt = CLIB.btSoftBody_getFdbvt,
	BvhTriangleMeshShape_serializeSingleBvh = CLIB.btBvhTriangleMeshShape_serializeSingleBvh,
	SoftBody_getCollisionDisabledObjects = CLIB.btSoftBody_getCollisionDisabledObjects,
	GImpactMeshShapePart_new2 = CLIB.btGImpactMeshShapePart_new2,
	SoftBody_getClusterConnectivity = CLIB.btSoftBody_getClusterConnectivity,
	MultiSphereShape_getSphereCount = CLIB.btMultiSphereShape_getSphereCount,
	SimulationIslandManager_buildIslands = CLIB.btSimulationIslandManager_buildIslands,
	MultiBody_addJointTorqueMultiDof = CLIB.btMultiBody_addJointTorqueMultiDof,
	SliderConstraint_testAngLimits = CLIB.btSliderConstraint_testAngLimits,
	Box2dBox2dCollisionAlgorithm_CreateFunc_new = CLIB.btBox2dBox2dCollisionAlgorithm_CreateFunc_new,
	SoftBody_getBounds = CLIB.btSoftBody_getBounds,
	MultiBody_updateCollisionObjectWorldTransforms = CLIB.btMultiBody_updateCollisionObjectWorldTransforms,
	RotationalLimitMotor_isLimited = CLIB.btRotationalLimitMotor_isLimited,
	SoftBody_getAnchors = CLIB.btSoftBody_getAnchors,
	SoftBody_generateClusters2 = CLIB.btSoftBody_generateClusters2,
	MultiBodySliderConstraint_getFrameInA = CLIB.btMultiBodySliderConstraint_getFrameInA,
	PolarDecomposition_new = CLIB.btPolarDecomposition_new,
	SoftBody_Joint_getErp = CLIB.btSoftBody_Joint_getErp,
	MultiBodySolverConstraint_getOrgDofIndex = CLIB.btMultiBodySolverConstraint_getOrgDofIndex,
	SoftBody_generateClusters = CLIB.btSoftBody_generateClusters,
	ConeTwistConstraint_getInfo2NonVirtual = CLIB.btConeTwistConstraint_getInfo2NonVirtual,
	MultiBodyFixedConstraint_setPivotInA = CLIB.btMultiBodyFixedConstraint_setPivotInA,
	ManifoldPoint_getDistance1 = CLIB.btManifoldPoint_getDistance1,
	ConstraintSolver_solveGroup = CLIB.btConstraintSolver_solveGroup,
	MultiBodyConstraint_getNumRows = CLIB.btMultiBodyConstraint_getNumRows,
	CollisionObject_getUserIndex = CLIB.btCollisionObject_getUserIndex,
	SoftBody_SolverState_setSdt = CLIB.btSoftBody_SolverState_setSdt,
	Box2dShape_new2 = CLIB.btBox2dShape_new2,
	SoftBody_cutLink2 = CLIB.btSoftBody_cutLink2,
	SoftBody_clusterVImpulse = CLIB.btSoftBody_clusterVImpulse,
	TranslationalLimitMotor2_getMotorERP = CLIB.btTranslationalLimitMotor2_getMotorERP,
	SoftBody_clusterVAImpulse = CLIB.btSoftBody_clusterVAImpulse,
	RigidBody_btRigidBodyConstructionInfo_setCollisionShape = CLIB.btRigidBody_btRigidBodyConstructionInfo_setCollisionShape,
	SoftBody_clusterDImpulse = CLIB.btSoftBody_clusterDImpulse,
	SoftBody_clusterDCImpulse = CLIB.btSoftBody_clusterDCImpulse,
	SoftBody_clusterDAImpulse = CLIB.btSoftBody_clusterDAImpulse,
	TypedConstraint_btConstraintInfo2_getRowskip = CLIB.btTypedConstraint_btConstraintInfo2_getRowskip,
	SoftBody_clusterCom2 = CLIB.btSoftBody_clusterCom2,
	Point2PointConstraint_getFlags = CLIB.btPoint2PointConstraint_getFlags,
	SoftBody_clusterAImpulse = CLIB.btSoftBody_clusterAImpulse,
	MultiBodyPoint2Point_new = CLIB.btMultiBodyPoint2Point_new,
	MultiBody_getJointVel = CLIB.btMultiBody_getJointVel,
	SoftBody_Link_new2 = CLIB.btSoftBody_Link_new2,
	SoftBody_cleanupClusters = CLIB.btSoftBody_cleanupClusters,
	SoftBody_checkLink2 = CLIB.btSoftBody_checkLink2,
	SoftBody_checkFace = CLIB.btSoftBody_checkFace,
	SoftBody_CJoint_getLife = CLIB.btSoftBody_CJoint_getLife,
	Dbvt_sStkCLN_getParent = CLIB.btDbvt_sStkCLN_getParent,
	AlignedObjectArray_btIndexedMesh_push_back = CLIB.btAlignedObjectArray_btIndexedMesh_push_back,
	CollisionWorld_LocalConvexResult_setHitFraction = CLIB.btCollisionWorld_LocalConvexResult_setHitFraction,
	Generic6DofConstraint_setLinearUpperLimit = CLIB.btGeneric6DofConstraint_setLinearUpperLimit,
	GImpactCollisionAlgorithm_setFace0 = CLIB.btGImpactCollisionAlgorithm_setFace0,
	MultiBody_localPosToWorld = CLIB.btMultiBody_localPosToWorld,
	RigidBody_btRigidBodyConstructionInfo_setStartWorldTransform = CLIB.btRigidBody_btRigidBodyConstructionInfo_setStartWorldTransform,
	ManifoldPoint_setPositionWorldOnA = CLIB.btManifoldPoint_setPositionWorldOnA,
	GImpactMeshShapePart_TrimeshPrimitiveManager_getMeshInterface = CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_getMeshInterface,
	SoftBody_getSst = CLIB.btSoftBody_getSst,
	DynamicsWorld_removeAction = CLIB.btDynamicsWorld_removeAction,
	SoftBody_appendNote4 = CLIB.btSoftBody_appendNote4,
	PointCollector_setNormalOnBInWorld = CLIB.btPointCollector_setNormalOnBInWorld,
	SoftBody_appendNote3 = CLIB.btSoftBody_appendNote3,
	MultiBody_useRK4Integration = CLIB.btMultiBody_useRK4Integration,
	ManifoldResult_getClosestPointDistanceThreshold = CLIB.btManifoldResult_getClosestPointDistanceThreshold,
	Generic6DofConstraint_setUseSolveConstraintObsolete = CLIB.btGeneric6DofConstraint_setUseSolveConstraintObsolete,
	SoftBody_Joint_Type = CLIB.btSoftBody_Joint_Type,
	MultiBodyConstraint_jacobianB = CLIB.btMultiBodyConstraint_jacobianB,
	SoftBody_Pose_getWgh = CLIB.btSoftBody_Pose_getWgh,
	SoftBody_Joint_getBodies = CLIB.btSoftBody_Joint_getBodies,
	SoftBody_Config_getKVC = CLIB.btSoftBody_Config_getKVC,
	SoftBody_appendLink2 = CLIB.btSoftBody_appendLink2,
	CollisionShape_isNonMoving = CLIB.btCollisionShape_isNonMoving,
	SoftBody_sCti_getColObj = CLIB.btSoftBody_sCti_getColObj,
	CollisionObject_getAnisotropicFriction = CLIB.btCollisionObject_getAnisotropicFriction,
	SortedOverlappingPairCache_new = CLIB.btSortedOverlappingPairCache_new,
	QuantizedBvhTree_getRightNode = CLIB.btQuantizedBvhTree_getRightNode,
	SliderConstraint_setRestitutionDirLin = CLIB.btSliderConstraint_setRestitutionDirLin,
	OverlappingPairCache_sortOverlappingPairs = CLIB.btOverlappingPairCache_sortOverlappingPairs,
	MultiBodyConstraint_updateJacobianSizes = CLIB.btMultiBodyConstraint_updateJacobianSizes,
	GImpactQuantizedBvh_hasHierarchy = CLIB.btGImpactQuantizedBvh_hasHierarchy,
	AxisSweep3_unQuantize = CLIB.btAxisSweep3_unQuantize,
	SoftBody_appendLinearJoint2 = CLIB.btSoftBody_appendLinearJoint2,
	SoftBody_AJoint_Specs_new = CLIB.btSoftBody_AJoint_Specs_new,
	QuantizedBvh_serialize = CLIB.btQuantizedBvh_serialize,
	SoftBody_Anchor_getNode = CLIB.btSoftBody_Anchor_getNode,
	SoftBody_appendFace = CLIB.btSoftBody_appendFace,
	BroadphaseProxy_isPolyhedral = CLIB.btBroadphaseProxy_isPolyhedral,
	VoronoiSimplexSolver_setEqualVertexThreshold = CLIB.btVoronoiSimplexSolver_setEqualVertexThreshold,
	ManifoldPoint_getContactMotion2 = CLIB.btManifoldPoint_getContactMotion2,
	SoftBody_appendAngularJoint = CLIB.btSoftBody_appendAngularJoint,
	SoftBody_appendAnchor2 = CLIB.btSoftBody_appendAnchor2,
	SoftBody_appendAnchor = CLIB.btSoftBody_appendAnchor,
	TypedConstraint_btConstraintInfo1_delete = CLIB.btTypedConstraint_btConstraintInfo1_delete,
	SoftBody_AJoint_Specs_getIcontrol = CLIB.btSoftBody_AJoint_Specs_getIcontrol,
	SphereSphereCollisionAlgorithm_new = CLIB.btSphereSphereCollisionAlgorithm_new,
	CompoundShape_calculatePrincipalAxisTransform = CLIB.btCompoundShape_calculatePrincipalAxisTransform,
	Generic6DofSpring2Constraint_matrixToEulerYXZ = CLIB.btGeneric6DofSpring2Constraint_matrixToEulerYXZ,
	DispatcherInfo_setUseContinuous = CLIB.btDispatcherInfo_setUseContinuous,
	ConvexPolyhedron_testContainment = CLIB.btConvexPolyhedron_testContainment,
	SoftBody_new = CLIB.btSoftBody_new,
	ConvexPolyhedron_getME = CLIB.btConvexPolyhedron_getME,
	SoftBody_Tetra_setLeaf = CLIB.btSoftBody_Tetra_setLeaf,
	SoftBody_Tetra_setC2 = CLIB.btSoftBody_Tetra_setC2,
	SoftBody_Tetra_setC1 = CLIB.btSoftBody_Tetra_setC1,
	SoftBody_Tetra_getN = CLIB.btSoftBody_Tetra_getN,
	BoxShape_new3 = CLIB.btBoxShape_new3,
	CollisionObject_removeCustomDebugColor = CLIB.btCollisionObject_removeCustomDebugColor,
	Dbvt_ICollide_AllLeaves = CLIB.btDbvt_ICollide_AllLeaves,
	SoftBody_Node_getX = CLIB.btSoftBody_Node_getX,
	ConvexInternalAabbCachingShape_recalcLocalAabb = CLIB.btConvexInternalAabbCachingShape_recalcLocalAabb,
	SoftBody_Tetra_getC0 = CLIB.btSoftBody_Tetra_getC0,
	DiscreteCollisionDetectorInterface_getClosestPoints = CLIB.btDiscreteCollisionDetectorInterface_getClosestPoints,
	SoftBody_sRayCast_delete = CLIB.btSoftBody_sRayCast_delete,
	SoftBody_sRayCast_setIndex = CLIB.btSoftBody_sRayCast_setIndex,
	RigidBody_computeGyroscopicImpulseImplicit_Body = CLIB.btRigidBody_computeGyroscopicImpulseImplicit_Body,
	SoftBody_sRayCast_setFraction = CLIB.btSoftBody_sRayCast_setFraction,
	RigidBody_getTotalTorque = CLIB.btRigidBody_getTotalTorque,
	SoftBody_sRayCast_setFeature = CLIB.btSoftBody_sRayCast_setFeature,
	SoftBody_sRayCast_setBody = CLIB.btSoftBody_sRayCast_setBody,
	MultimaterialTriangleMeshShape_new2 = CLIB.btMultimaterialTriangleMeshShape_new2,
	SoftBody_sRayCast_getIndex = CLIB.btSoftBody_sRayCast_getIndex,
	SoftBody_sRayCast_getFraction = CLIB.btSoftBody_sRayCast_getFraction,
	MLCPSolver_getNumFallbacks = CLIB.btMLCPSolver_getNumFallbacks,
	MultiBodySolverConstraint_getMultiBodyA = CLIB.btMultiBodySolverConstraint_getMultiBodyA,
	SoftBody_SolverState_setVelmrg = CLIB.btSoftBody_SolverState_setVelmrg,
	DispatcherInfo_getEnableSatConvex = CLIB.btDispatcherInfo_getEnableSatConvex,
	SoftBody_dampClusters = CLIB.btSoftBody_dampClusters,
	Face_getPlane = CLIB.btFace_getPlane,
	DbvtProxy_getLinks = CLIB.btDbvtProxy_getLinks,
	DbvtNode_getParent = CLIB.btDbvtNode_getParent,
	SoftBody_SolverState_setRadmrg = CLIB.btSoftBody_SolverState_setRadmrg,
	SoftBody_SolverState_setIsdt = CLIB.btSoftBody_SolverState_setIsdt,
	SoftBody_SolverState_getVelmrg = CLIB.btSoftBody_SolverState_getVelmrg,
	RigidBody_getLinearFactor = CLIB.btRigidBody_getLinearFactor,
	AlignedObjectArray_btCollisionObjectPtr_findLinearSearch2 = CLIB.btAlignedObjectArray_btCollisionObjectPtr_findLinearSearch2,
	SoftBody_SolverState_getUpdmrg = CLIB.btSoftBody_SolverState_getUpdmrg,
	SoftBody_SolverState_getSdt = CLIB.btSoftBody_SolverState_getSdt,
	CollisionObject_setWorldTransform = CLIB.btCollisionObject_setWorldTransform,
	SoftBody_SolverState_getRadmrg = CLIB.btSoftBody_SolverState_getRadmrg,
	SoftBody_SolverState_getIsdt = CLIB.btSoftBody_SolverState_getIsdt,
	BoxShape_getHalfExtentsWithoutMargin = CLIB.btBoxShape_getHalfExtentsWithoutMargin,
	SoftBody_sCti_delete = CLIB.btSoftBody_sCti_delete,
	SoftBody_sCti_setOffset = CLIB.btSoftBody_sCti_setOffset,
	SoftBody_sCti_setNormal = CLIB.btSoftBody_sCti_setNormal,
	SoftBody_sCti_setColObj = CLIB.btSoftBody_sCti_setColObj,
	SphereTriangleCollisionAlgorithm_CreateFunc_new = CLIB.btSphereTriangleCollisionAlgorithm_CreateFunc_new,
	SoftBody_Body_delete = CLIB.btSoftBody_Body_delete,
	SoftBody_sCti_new = CLIB.btSoftBody_sCti_new,
	SoftBody_Config_setKDG = CLIB.btSoftBody_Config_setKDG,
	SoftBody_SContact_setWeights = CLIB.btSoftBody_SContact_setWeights,
	SoftBody_SContact_setNormal = CLIB.btSoftBody_SContact_setNormal,
	SoftBody_SContact_setNode = CLIB.btSoftBody_SContact_setNode,
	SoftBody_SContact_setMargin = CLIB.btSoftBody_SContact_setMargin,
	SoftBody_SContact_setFriction = CLIB.btSoftBody_SContact_setFriction,
	SoftBody_SContact_setFace = CLIB.btSoftBody_SContact_setFace,
	SoftBody_AJoint_IControl_delete = CLIB.btSoftBody_AJoint_IControl_delete,
	BroadphaseProxy_getClientObject = CLIB.btBroadphaseProxy_getClientObject,
	SoftBody_SContact_getNormal = CLIB.btSoftBody_SContact_getNormal,
	Chunk_setOldPtr = CLIB.btChunk_setOldPtr,
	IDebugDraw_delete = CLIB.btIDebugDraw_delete,
	SoftBody_Joint_getRefs = CLIB.btSoftBody_Joint_getRefs,
	SoftBody_SContact_getNode = CLIB.btSoftBody_SContact_getNode,
	CollisionWorld_ClosestRayResultCallback_setRayToWorld = CLIB.btCollisionWorld_ClosestRayResultCallback_setRayToWorld,
	SoftBody_SContact_getFriction = CLIB.btSoftBody_SContact_getFriction,
	SoftBody_SContact_getCfm = CLIB.btSoftBody_SContact_getCfm,
	MultiBodySolverConstraint_getLowerLimit = CLIB.btMultiBodySolverConstraint_getLowerLimit,
	GImpactMeshShape_getMeshPart = CLIB.btGImpactMeshShape_getMeshPart,
	SoftBody_Link_setC0 = CLIB.btSoftBody_Link_setC0,
	SoftBody_RContact_setNode = CLIB.btSoftBody_RContact_setNode,
	RotationalLimitMotor_setCurrentPosition = CLIB.btRotationalLimitMotor_setCurrentPosition,
	SoftBody_Pose_getAqq = CLIB.btSoftBody_Pose_getAqq,
	TypedConstraint_needsFeedback = CLIB.btTypedConstraint_needsFeedback,
	SoftBody_RContact_setC3 = CLIB.btSoftBody_RContact_setC3,
	SoftBody_RContact_setC2 = CLIB.btSoftBody_RContact_setC2,
	SoftBody_RContact_setC1 = CLIB.btSoftBody_RContact_setC1,
	SoftBody_RContact_setC0 = CLIB.btSoftBody_RContact_setC0,
	MultiBodyDynamicsWorld_getNumMultiBodyConstraints = CLIB.btMultiBodyDynamicsWorld_getNumMultiBodyConstraints,
	CapsuleShape_getUpAxis = CLIB.btCapsuleShape_getUpAxis,
	ContactSolverInfoData_getLinearSlop = CLIB.btContactSolverInfoData_getLinearSlop,
	ManifoldResult_calculateCombinedFriction = CLIB.btManifoldResult_calculateCombinedFriction,
	SoftBody_RContact_getNode = CLIB.btSoftBody_RContact_getNode,
	CompoundShape_getDynamicAabbTree = CLIB.btCompoundShape_getDynamicAabbTree,
	CollisionAlgorithmConstructionInfo_getDispatcher1 = CLIB.btCollisionAlgorithmConstructionInfo_getDispatcher1,
	SoftBody_RContact_getC3 = CLIB.btSoftBody_RContact_getC3,
	GhostObject_removeOverlappingObjectInternal = CLIB.btGhostObject_removeOverlappingObjectInternal,
	ConvexConvexAlgorithm_CreateFunc_getPdSolver = CLIB.btConvexConvexAlgorithm_CreateFunc_getPdSolver,
	SoftBody_RContact_getC2 = CLIB.btSoftBody_RContact_getC2,
	GImpactCollisionAlgorithm_gimpact_vs_shape = CLIB.btGImpactCollisionAlgorithm_gimpact_vs_shape,
	Dbvt_sStkCLN_delete = CLIB.btDbvt_sStkCLN_delete,
	SoftBody_RayFromToCaster_setTests = CLIB.btSoftBody_RayFromToCaster_setTests,
	SoftBody_Note_getText = CLIB.btSoftBody_Note_getText,
	Dispatcher_needsCollision = CLIB.btDispatcher_needsCollision,
	SoftBody_PSolve_Links = CLIB.btSoftBody_PSolve_Links,
	BroadphaseProxy_isCompound = CLIB.btBroadphaseProxy_isCompound,
	SoftBody_RayFromToCaster_setMint = CLIB.btSoftBody_RayFromToCaster_setMint,
	SoftBody_RayFromToCaster_setFace = CLIB.btSoftBody_RayFromToCaster_setFace,
	CollisionWorld_ContactResultCallbackWrapper_new = CLIB.btCollisionWorld_ContactResultCallbackWrapper_new,
	SoftBody_RayFromToCaster_rayFromToTriangle2 = CLIB.btSoftBody_RayFromToCaster_rayFromToTriangle2,
	SoftBody_RayFromToCaster_getRayTo = CLIB.btSoftBody_RayFromToCaster_getRayTo,
	MultiBodyConstraint_getMultiBodyA = CLIB.btMultiBodyConstraint_getMultiBodyA,
	SoftBody_CJoint_getRpos = CLIB.btSoftBody_CJoint_getRpos,
	BroadphasePair_delete = CLIB.btBroadphasePair_delete,
	RotationalLimitMotor_setStopERP = CLIB.btRotationalLimitMotor_setStopERP,
	PersistentManifold_getBody0 = CLIB.btPersistentManifold_getBody0,
	TriangleMesh_getWeldingThreshold = CLIB.btTriangleMesh_getWeldingThreshold,
	SoftBody_Element_setTag = CLIB.btSoftBody_Element_setTag,
	SoftBody_Pose_setVolume = CLIB.btSoftBody_Pose_setVolume,
	MultiBody_addLinkConstraintForce = CLIB.btMultiBody_addLinkConstraintForce,
	AxisSweep3_updateHandle = CLIB.btAxisSweep3_updateHandle,
	SoftBody_Pose_setRot = CLIB.btSoftBody_Pose_setRot,
	SoftBody_Pose_setCom = CLIB.btSoftBody_Pose_setCom,
	HingeConstraint_setUseFrameOffset = CLIB.btHingeConstraint_setUseFrameOffset,
	SoftBody_Pose_setBvolume = CLIB.btSoftBody_Pose_setBvolume,
	SoftBody_AJoint_IControl_Default = CLIB.btSoftBody_AJoint_IControl_Default,
	SoftBody_Pose_setBframe = CLIB.btSoftBody_Pose_setBframe,
	SoftBody_Pose_setAqq = CLIB.btSoftBody_Pose_setAqq,
	CollisionWorld_getDebugDrawer = CLIB.btCollisionWorld_getDebugDrawer,
	AABB_overlapping_trans_cache = CLIB.btAABB_overlapping_trans_cache,
	SoftBody_Pose_getRot = CLIB.btSoftBody_Pose_getRot,
	BU_Simplex1to4_addVertex = CLIB.btBU_Simplex1to4_addVertex,
	MultibodyLink_getInertiaLocal = CLIB.btMultibodyLink_getInertiaLocal,
	SoftBody_Pose_getBframe = CLIB.btSoftBody_Pose_getBframe,
	MultibodyLink_setAxisBottom2 = CLIB.btMultibodyLink_setAxisBottom2,
	SoftBody_Note_setText = CLIB.btSoftBody_Note_setText,
	SoftBody_Note_setRank = CLIB.btSoftBody_Note_setRank,
	SoftBody_Note_setOffset = CLIB.btSoftBody_Note_setOffset,
	SoftBody_RayFromToCaster_setRayNormalizedDirection = CLIB.btSoftBody_RayFromToCaster_setRayNormalizedDirection,
	CollisionDispatcher_defaultNearCallback = CLIB.btCollisionDispatcher_defaultNearCallback,
	CollisionWorld_RayResultCallback_setCollisionObject = CLIB.btCollisionWorld_RayResultCallback_setCollisionObject,
	SoftBody_Face_getLeaf = CLIB.btSoftBody_Face_getLeaf,
	SoftBody_Note_getOffset = CLIB.btSoftBody_Note_getOffset,
	SoftBody_Note_getNodes = CLIB.btSoftBody_Note_getNodes,
	CollisionWorld_ConvexResultCallback_addSingleResult = CLIB.btCollisionWorld_ConvexResultCallback_addSingleResult,
	MLCPSolver_setNumFallbacks = CLIB.btMLCPSolver_setNumFallbacks,
	TranslationalLimitMotor_isLimited = CLIB.btTranslationalLimitMotor_isLimited,
	GjkPairDetector_setPenetrationDepthSolver = CLIB.btGjkPairDetector_setPenetrationDepthSolver,
	MultiBody_getMaxCoordinateVelocity = CLIB.btMultiBody_getMaxCoordinateVelocity,
	SoftBody_Node_setV = CLIB.btSoftBody_Node_setV,
	SoftBody_Node_setLeaf = CLIB.btSoftBody_Node_setLeaf,
	SoftBody_Node_setIm = CLIB.btSoftBody_Node_setIm,
	SoftBody_Node_setF = CLIB.btSoftBody_Node_setF,
	SoftBody_Node_setBattach = CLIB.btSoftBody_Node_setBattach,
	SoftBody_Node_setArea = CLIB.btSoftBody_Node_setArea,
	BroadphasePair_new2 = CLIB.btBroadphasePair_new2,
	ConeTwistConstraint_getTwistAngle = CLIB.btConeTwistConstraint_getTwistAngle,
	CollisionWorld_ClosestConvexResultCallback_getConvexToWorld = CLIB.btCollisionWorld_ClosestConvexResultCallback_getConvexToWorld,
	SliderConstraint_getDampingLimAng = CLIB.btSliderConstraint_getDampingLimAng,
	SoftBody_Node_getV = CLIB.btSoftBody_Node_getV,
	SoftBody_Node_getQ = CLIB.btSoftBody_Node_getQ,
	GearConstraint_setAxisA = CLIB.btGearConstraint_setAxisA,
	SliderConstraint_setSoftnessLimLin = CLIB.btSliderConstraint_setSoftnessLimLin,
	SliderConstraint_setLowerLinLimit = CLIB.btSliderConstraint_setLowerLinLimit,
	SoftBody_Node_getF = CLIB.btSoftBody_Node_getF,
	SoftRigidDynamicsWorld_getWorldInfo = CLIB.btSoftRigidDynamicsWorld_getWorldInfo,
	SoftBody_Material_setKVST = CLIB.btSoftBody_Material_setKVST,
	SoftBody_Material_setKLST = CLIB.btSoftBody_Material_setKLST,
	SoftBody_Material_setKAST = CLIB.btSoftBody_Material_setKAST,
	SoftBody_Material_setFlags = CLIB.btSoftBody_Material_setFlags,
	SoftBody_Material_getKVST = CLIB.btSoftBody_Material_getKVST,
	SoftBody_Material_getFlags = CLIB.btSoftBody_Material_getFlags,
	SoftBody_predictMotion = CLIB.btSoftBody_predictMotion,
	SoftBody_LJoint_Specs_setPosition = CLIB.btSoftBody_LJoint_Specs_setPosition,
	BvhTriangleMeshShape_refitTree = CLIB.btBvhTriangleMeshShape_refitTree,
	AABB_setMin = CLIB.btAABB_setMin,
	SoftBody_LJoint_Specs_getPosition = CLIB.btSoftBody_LJoint_Specs_getPosition,
	SoftBody_LJoint_Specs_new = CLIB.btSoftBody_LJoint_Specs_new,
	ContactSolverInfoData_getErp2 = CLIB.btContactSolverInfoData_getErp2,
	CylinderShape_new2 = CLIB.btCylinderShape_new2,
	Generic6DofConstraint_getTranslationalLimitMotor = CLIB.btGeneric6DofConstraint_getTranslationalLimitMotor,
	SoftBody_Link_setRl = CLIB.btSoftBody_Link_setRl,
	SoftBody_Link_setC3 = CLIB.btSoftBody_Link_setC3,
	Generic6DofConstraint_setUseLinearReferenceFrameA = CLIB.btGeneric6DofConstraint_setUseLinearReferenceFrameA,
	SoftBody_Link_setC2 = CLIB.btSoftBody_Link_setC2,
	SoftBody_Link_setC1 = CLIB.btSoftBody_Link_setC1,
	SoftBody_RContact_delete = CLIB.btSoftBody_RContact_delete,
	Box2dShape_new3 = CLIB.btBox2dShape_new3,
	SoftBody_Link_setBbending = CLIB.btSoftBody_Link_setBbending,
	SoftBody_Link_getN = CLIB.btSoftBody_Link_getN,
	CollisionWorld_ClosestConvexResultCallback_getConvexFromWorld = CLIB.btCollisionWorld_ClosestConvexResultCallback_getConvexFromWorld,
	DiscreteDynamicsWorld_setNumTasks = CLIB.btDiscreteDynamicsWorld_setNumTasks,
	DiscreteDynamicsWorld_getSynchronizeAllMotionStates = CLIB.btDiscreteDynamicsWorld_getSynchronizeAllMotionStates,
	RigidBody_getAabb = CLIB.btRigidBody_getAabb,
	SoftBody_Link_getC1 = CLIB.btSoftBody_Link_getC1,
	SoftBody_Link_getC0 = CLIB.btSoftBody_Link_getC0,
	ConvexCast_CastResult_getAllowedPenetration = CLIB.btConvexCast_CastResult_getAllowedPenetration,
	ContactSolverInfoData_getRestingContactRestitutionThreshold = CLIB.btContactSolverInfoData_getRestingContactRestitutionThreshold,
	CollisionWorld_LocalRayResult_getHitFraction = CLIB.btCollisionWorld_LocalRayResult_getHitFraction,
	QuantizedBvhTree_new = CLIB.btQuantizedBvhTree_new,
	Generic6DofSpring2Constraint_matrixToEulerZYX = CLIB.btGeneric6DofSpring2Constraint_matrixToEulerZYX,
	CompoundCollisionAlgorithm_CreateFunc_new = CLIB.btCompoundCollisionAlgorithm_CreateFunc_new,
	SoftBody_Link_new = CLIB.btSoftBody_Link_new,
	MultibodyLink_getPosVarCount = CLIB.btMultibodyLink_getPosVarCount,
	AngularLimit_getSign = CLIB.btAngularLimit_getSign,
	SoftBody_appendMaterial = CLIB.btSoftBody_appendMaterial,
	SoftBody_Joint_Solve = CLIB.btSoftBody_Joint_Solve,
	CollisionObjectWrapper_getWorldTransform = CLIB.btCollisionObjectWrapper_getWorldTransform,
	MultiBodyConstraint_debugDraw = CLIB.btMultiBodyConstraint_debugDraw,
	SoftBody_Joint_setSdrift = CLIB.btSoftBody_Joint_setSdrift,
	SoftBody_Joint_setMassmatrix = CLIB.btSoftBody_Joint_setMassmatrix,
	MultiBody_fillConstraintJacobianMultiDof = CLIB.btMultiBody_fillConstraintJacobianMultiDof,
	CollisionWorld_LocalRayResult_getCollisionObject = CLIB.btCollisionWorld_LocalRayResult_getCollisionObject,
	SoftBody_Joint_setErp = CLIB.btSoftBody_Joint_setErp,
	SoftBody_Joint_setDrift = CLIB.btSoftBody_Joint_setDrift,
	Generic6DofSpring2Constraint_calculateTransforms2 = CLIB.btGeneric6DofSpring2Constraint_calculateTransforms2,
	SoftBody_Joint_setDelete = CLIB.btSoftBody_Joint_setDelete,
	SoftBody_Joint_setCfm = CLIB.btSoftBody_Joint_setCfm,
	SoftBody_Joint_Prepare = CLIB.btSoftBody_Joint_Prepare,
	SphereBoxCollisionAlgorithm_getSphereDistance = CLIB.btSphereBoxCollisionAlgorithm_getSphereDistance,
	SoftBody_Joint_getSdrift = CLIB.btSoftBody_Joint_getSdrift,
	HingeAccumulatedAngleConstraint_getAccumulatedHingeAngle = CLIB.btHingeAccumulatedAngleConstraint_getAccumulatedHingeAngle,
	SoftBody_Anchor_setC2 = CLIB.btSoftBody_Anchor_setC2,
	SoftBody_Joint_getCfm = CLIB.btSoftBody_Joint_getCfm,
	HingeConstraint_getFlags = CLIB.btHingeConstraint_getFlags,
	SoftBody_appendLink3 = CLIB.btSoftBody_appendLink3,
	GImpactBvh_getNodeData = CLIB.btGImpactBvh_getNodeData,
	SoftBody_Joint_Specs_getSplit = CLIB.btSoftBody_Joint_Specs_getSplit,
	SoftBody_Joint_Specs_setSplit = CLIB.btSoftBody_Joint_Specs_setSplit,
	DbvtNode_isinternal = CLIB.btDbvtNode_isinternal,
	DynamicsWorld_getWorldUserInfo = CLIB.btDynamicsWorld_getWorldUserInfo,
	SoftBody_staticSolve = CLIB.btSoftBody_staticSolve,
	AlignedObjectArray_btVector3_push_back2 = CLIB.btAlignedObjectArray_btVector3_push_back2,
	CollisionWorld_getDispatchInfo = CLIB.btCollisionWorld_getDispatchInfo,
	AngularLimit_delete = CLIB.btAngularLimit_delete,
	SoftBody_Impulse_setVelocity = CLIB.btSoftBody_Impulse_setVelocity,
	Generic6DofConstraint_setUseFrameOffset = CLIB.btGeneric6DofConstraint_setUseFrameOffset,
	MultiBodyJointLimitConstraint_new = CLIB.btMultiBodyJointLimitConstraint_new,
	SoftBody_Impulse_setDrift = CLIB.btSoftBody_Impulse_setDrift,
	SoftBody_Impulse_setAsVelocity = CLIB.btSoftBody_Impulse_setAsVelocity,
	OverlappingPairCallback_addOverlappingPair = CLIB.btOverlappingPairCallback_addOverlappingPair,
	TranslationalLimitMotor2_getLowerLimit = CLIB.btTranslationalLimitMotor2_getLowerLimit,
	Dbvt_optimizeIncremental = CLIB.btDbvt_optimizeIncremental,
	GImpactShapeInterface_getBoxSet = CLIB.btGImpactShapeInterface_getBoxSet,
	BoxShape_new2 = CLIB.btBoxShape_new2,
	GImpactCollisionAlgorithm_registerAlgorithm = CLIB.btGImpactCollisionAlgorithm_registerAlgorithm,
	DispatcherInfo_getTimeOfImpact = CLIB.btDispatcherInfo_getTimeOfImpact,
	ConvexCast_CastResult_getDebugDrawer = CLIB.btConvexCast_CastResult_getDebugDrawer,
	MultibodyLink_setCfgOffset = CLIB.btMultibodyLink_setCfgOffset,
	SoftBody_Impulse_new = CLIB.btSoftBody_Impulse_new,
	SoftBody_getFaceVertexNormalData = CLIB.btSoftBody_getFaceVertexNormalData,
	SoftBody_ImplicitFn_Eval = CLIB.btSoftBody_ImplicitFn_Eval,
	ConvexConcaveCollisionAlgorithm_CreateFunc_new = CLIB.btConvexConcaveCollisionAlgorithm_CreateFunc_new,
	SoftBody_Feature_setMaterial = CLIB.btSoftBody_Feature_setMaterial,
	BvhTree_getEscapeNodeIndex = CLIB.btBvhTree_getEscapeNodeIndex,
	CollisionWorld_updateSingleAabb = CLIB.btCollisionWorld_updateSingleAabb,
	DbvtNode_new = CLIB.btDbvtNode_new,
	ConvexHullShape_optimizeConvexHull = CLIB.btConvexHullShape_optimizeConvexHull,
	SoftBody_Face_setRa = CLIB.btSoftBody_Face_setRa,
	TriangleMesh_getUse32bitIndices = CLIB.btTriangleMesh_getUse32bitIndices,
	SoftBody_Face_setLeaf = CLIB.btSoftBody_Face_setLeaf,
	OptimizedBvh_updateBvhNodes = CLIB.btOptimizedBvh_updateBvhNodes,
	CollisionShape_getAnisotropicRollingFrictionDirection = CLIB.btCollisionShape_getAnisotropicRollingFrictionDirection,
	GImpactQuantizedBvh_getLeftNode = CLIB.btGImpactQuantizedBvh_getLeftNode,
	SoftBody_Face_getRa = CLIB.btSoftBody_Face_getRa,
	SoftBody_Face_getNormal = CLIB.btSoftBody_Face_getNormal,
	SoftBody_Face_getN = CLIB.btSoftBody_Face_getN,
	PairSet_push_pair = CLIB.btPairSet_push_pair,
	SoftBody_RayFromToCaster_new = CLIB.btSoftBody_RayFromToCaster_new,
	SoftBody_Body_xform = CLIB.btSoftBody_Body_xform,
	MultibodyLink_getAppliedForce = CLIB.btMultibodyLink_getAppliedForce,
	SoftBody_Config_setViterations = CLIB.btSoftBody_Config_setViterations,
	SoftBody_Config_setTimescale = CLIB.btSoftBody_Config_setTimescale,
	SoftBody_Config_setPiterations = CLIB.btSoftBody_Config_setPiterations,
	GhostObject_new = CLIB.btGhostObject_new,
	SoftBody_Config_setMaxvolume = CLIB.btSoftBody_Config_setMaxvolume,
	SoftBody_Config_setKVCF = CLIB.btSoftBody_Config_setKVCF,
	SoftBody_Config_setKVC = CLIB.btSoftBody_Config_setKVC,
	BroadphasePair_setPProxy1 = CLIB.btBroadphasePair_setPProxy1,
	SoftBody_Config_setKSSHR_CL = CLIB.btSoftBody_Config_setKSSHR_CL,
	SoftBody_Config_setKSS_SPLT_CL = CLIB.btSoftBody_Config_setKSS_SPLT_CL,
	MultibodyLink_setPosVarCount = CLIB.btMultibodyLink_setPosVarCount,
	GImpactMeshShapePart_TrimeshPrimitiveManager_setIndexstride = CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_setIndexstride,
	CollisionShape_getUserIndex = CLIB.btCollisionShape_getUserIndex,
	MultiBodyFixedConstraint_setFrameInB = CLIB.btMultiBodyFixedConstraint_setFrameInB,
	SoftBody_Config_getKCHR = CLIB.btSoftBody_Config_getKCHR,
	MultiBody_setCanSleep = CLIB.btMultiBody_setCanSleep,
	GhostObject_getNumOverlappingObjects = CLIB.btGhostObject_getNumOverlappingObjects,
	SoftBody_Config_setKSRHR_CL = CLIB.btSoftBody_Config_setKSRHR_CL,
	SoftBody_Config_setKSR_SPLT_CL = CLIB.btSoftBody_Config_setKSR_SPLT_CL,
	ConvexPolyhedron_getExtents = CLIB.btConvexPolyhedron_getExtents,
	SliderConstraint_setSoftnessDirLin = CLIB.btSliderConstraint_setSoftnessDirLin,
	TranslationalLimitMotor2_getMaxMotorForce = CLIB.btTranslationalLimitMotor2_getMaxMotorForce,
	ConvexPointCloudShape_new2 = CLIB.btConvexPointCloudShape_new2,
	SoftBody_Config_setKSKHR_CL = CLIB.btSoftBody_Config_setKSKHR_CL,
	CollisionShape_getName = CLIB.btCollisionShape_getName,
	SoftBody_Config_setKSK_SPLT_CL = CLIB.btSoftBody_Config_setKSK_SPLT_CL,
	SoftBody_Config_setKSHR = CLIB.btSoftBody_Config_setKSHR,
	DynamicsWorld_getSolverInfo = CLIB.btDynamicsWorld_getSolverInfo,
	SoftBody_Config_setKPR = CLIB.btSoftBody_Config_setKPR,
	SoftBody_Config_setKMT = CLIB.btSoftBody_Config_setKMT,
	SoftBody_updateLinkConstants = CLIB.btSoftBody_updateLinkConstants,
	SoftBody_Cluster_getMasses = CLIB.btSoftBody_Cluster_getMasses,
	TypedConstraint_btConstraintInfo2_getConstraintError = CLIB.btTypedConstraint_btConstraintInfo2_getConstraintError,
	SoftBody_Config_setKDP = CLIB.btSoftBody_Config_setKDP,
	ContinuousConvexCollision_new2 = CLIB.btContinuousConvexCollision_new2,
	SoftBody_SContact_delete = CLIB.btSoftBody_SContact_delete,
	SoftBody_Config_setKDF = CLIB.btSoftBody_Config_setKDF,
	SoftBody_Config_setKCHR = CLIB.btSoftBody_Config_setKCHR,
	BvhTree_build_tree = CLIB.btBvhTree_build_tree,
	SoftBody_Config_setKAHR = CLIB.btSoftBody_Config_setKAHR,
	SoftBody_Config_setDiterations = CLIB.btSoftBody_Config_setDiterations,
	BvhTree_getNodeCount = CLIB.btBvhTree_getNodeCount,
	SoftBody_Config_setCollisions = CLIB.btSoftBody_Config_setCollisions,
	Dbvt_setOpath = CLIB.btDbvt_setOpath,
	SoftBody_Config_setCiterations = CLIB.btSoftBody_Config_setCiterations,
	SoftBody_Config_setAeromodel = CLIB.btSoftBody_Config_setAeromodel,
	SoftBody_Config_getVsequence = CLIB.btSoftBody_Config_getVsequence,
	SoftBody_Config_getViterations = CLIB.btSoftBody_Config_getViterations,
	ConeTwistConstraint_getFixThresh = CLIB.btConeTwistConstraint_getFixThresh,
	SoftBody_Config_getPsequence = CLIB.btSoftBody_Config_getPsequence,
	SoftBody_Config_getPiterations = CLIB.btSoftBody_Config_getPiterations,
	SoftBody_Config_getKVCF = CLIB.btSoftBody_Config_getKVCF,
	SoftBody_Cluster_getNodes = CLIB.btSoftBody_Cluster_getNodes,
	SoftBody_Config_getKSSHR_CL = CLIB.btSoftBody_Config_getKSSHR_CL,
	MultiBody_setBaseCollider = CLIB.btMultiBody_setBaseCollider,
	VoronoiSimplexSolver_setCachedValidClosest = CLIB.btVoronoiSimplexSolver_setCachedValidClosest,
	SoftBody_upcast = CLIB.btSoftBody_upcast,
	SoftBody_Config_getKSR_SPLT_CL = CLIB.btSoftBody_Config_getKSR_SPLT_CL,
	Dbvt_IClone_delete = CLIB.btDbvt_IClone_delete,
	SoftBody_Config_getKSKHR_CL = CLIB.btSoftBody_Config_getKSKHR_CL,
	SoftBody_Config_getKSHR = CLIB.btSoftBody_Config_getKSHR,
	SoftBody_Config_getKMT = CLIB.btSoftBody_Config_getKMT,
	DefaultCollisionConfiguration_setConvexConvexMultipointIterations = CLIB.btDefaultCollisionConfiguration_setConvexConvexMultipointIterations,
	SoftBody_Config_getKLF = CLIB.btSoftBody_Config_getKLF,
	Dbvt_sStkNPS_getNode = CLIB.btDbvt_sStkNPS_getNode,
	ManifoldPoint_getPositionWorldOnA = CLIB.btManifoldPoint_getPositionWorldOnA,
	SoftBody_Config_getKKHR = CLIB.btSoftBody_Config_getKKHR,
	DefaultCollisionConstructionInfo_setCustomCollisionAlgorithmMaxElementSize = CLIB.btDefaultCollisionConstructionInfo_setCustomCollisionAlgorithmMaxElementSize,
	SoftBody_Config_getKAHR = CLIB.btSoftBody_Config_getKAHR,
	SoftBody_Config_getDiterations = CLIB.btSoftBody_Config_getDiterations,
	Box2dShape_getHalfExtentsWithMargin = CLIB.btBox2dShape_getHalfExtentsWithMargin,
	TranslationalLimitMotor2_getCurrentLimit = CLIB.btTranslationalLimitMotor2_getCurrentLimit,
	SoftBody_Cluster_setSelfCollisionImpulseFactor = CLIB.btSoftBody_Cluster_setSelfCollisionImpulseFactor,
	SoftBody_Cluster_setNvimpulses = CLIB.btSoftBody_Cluster_setNvimpulses,
	SoftBody_Cluster_setNdimpulses = CLIB.btSoftBody_Cluster_setNdimpulses,
	VoronoiSimplexSolver_getEqualVertexThreshold = CLIB.btVoronoiSimplexSolver_getEqualVertexThreshold,
	DbvtAabbMm_FromPoints2 = CLIB.btDbvtAabbMm_FromPoints2,
	Dispatcher_dispatchAllCollisionPairs = CLIB.btDispatcher_dispatchAllCollisionPairs,
	SoftBody_Cluster_setMaxSelfCollisionImpulse = CLIB.btSoftBody_Cluster_setMaxSelfCollisionImpulse,
	SoftBody_Cluster_setMatching = CLIB.btSoftBody_Cluster_setMatching,
	ConvexShape_getPreferredPenetrationDirection = CLIB.btConvexShape_getPreferredPenetrationDirection,
	RotationalLimitMotor2_setStopCFM = CLIB.btRotationalLimitMotor2_setStopCFM,
	DispatcherInfo_getDebugDraw = CLIB.btDispatcherInfo_getDebugDraw,
	RigidBody_getAngularVelocity = CLIB.btRigidBody_getAngularVelocity,
	SoftBody_Cluster_setLv = CLIB.btSoftBody_Cluster_setLv,
	MultiBodySolverConstraint_getOriginalContactPoint = CLIB.btMultiBodySolverConstraint_getOriginalContactPoint,
	SoftBody_Cluster_setLocii = CLIB.btSoftBody_Cluster_setLocii,
	SoftBody_Cluster_setLeaf = CLIB.btSoftBody_Cluster_setLeaf,
	SoftBody_Cluster_setLdamping = CLIB.btSoftBody_Cluster_setLdamping,
	ManifoldPoint_setContactPointFlags = CLIB.btManifoldPoint_setContactPointFlags,
	CollisionObject_setIgnoreCollisionCheck = CLIB.btCollisionObject_setIgnoreCollisionCheck,
	SoftBody_Cluster_setInvwi = CLIB.btSoftBody_Cluster_setInvwi,
	SoftBody_Cluster_setImass = CLIB.btSoftBody_Cluster_setImass,
	SoftBody_Cluster_setIdmass = CLIB.btSoftBody_Cluster_setIdmass,
	GjkPairDetector_setCachedSeparatingAxis = CLIB.btGjkPairDetector_setCachedSeparatingAxis,
	DispatcherInfo_getAllowedCcdPenetration = CLIB.btDispatcherInfo_getAllowedCcdPenetration,
	Vector3_array_set = CLIB.btVector3_array_set,
	DiscreteCollisionDetectorInterface_ClosestPointInput_getTransformA = CLIB.btDiscreteCollisionDetectorInterface_ClosestPointInput_getTransformA,
	SoftBodySolver_updateSoftBodies = CLIB.btSoftBodySolver_updateSoftBodies,
	SoftBody_Cluster_setClusterIndex = CLIB.btSoftBody_Cluster_setClusterIndex,
	SoftBody_Cluster_setAv = CLIB.btSoftBody_Cluster_setAv,
	TranslationalLimitMotor_getLowerLimit = CLIB.btTranslationalLimitMotor_getLowerLimit,
	SoftBody_Cluster_setAdamping = CLIB.btSoftBody_Cluster_setAdamping,
	ContactSolverInfoData_getSplitImpulseTurnErp = CLIB.btContactSolverInfoData_getSplitImpulseTurnErp,
	SoftBody_Cluster_getVimpulses = CLIB.btSoftBody_Cluster_getVimpulses,
	ManifoldPoint_setCombinedContactDamping1 = CLIB.btManifoldPoint_setCombinedContactDamping1,
	SoftBody_Cluster_getNvimpulses = CLIB.btSoftBody_Cluster_getNvimpulses,
	AngularLimit_getHigh = CLIB.btAngularLimit_getHigh,
	SoftBody_Cluster_getNdamping = CLIB.btSoftBody_Cluster_getNdamping,
	SoftBody_Cluster_getMaxSelfCollisionImpulse = CLIB.btSoftBody_Cluster_getMaxSelfCollisionImpulse,
	AlignedObjectArray_btSoftBody_Face_size = CLIB.btAlignedObjectArray_btSoftBody_Face_size,
	SoftBody_Cluster_getLv = CLIB.btSoftBody_Cluster_getLv,
	BroadphaseInterface_delete = CLIB.btBroadphaseInterface_delete,
	CompoundShape_updateChildTransform = CLIB.btCompoundShape_updateChildTransform,
	SoftBody_Cluster_getLdamping = CLIB.btSoftBody_Cluster_getLdamping,
	ManifoldPoint_getLateralFrictionDir1 = CLIB.btManifoldPoint_getLateralFrictionDir1,
	MultiBodySolverConstraint_getRhsPenetration = CLIB.btMultiBodySolverConstraint_getRhsPenetration,
	MultiBody_checkMotionAndSleepIfRequired = CLIB.btMultiBody_checkMotionAndSleepIfRequired,
	SoftBody_Cluster_getInvwi = CLIB.btSoftBody_Cluster_getInvwi,
	HingeAccumulatedAngleConstraint_setAccumulatedHingeAngle = CLIB.btHingeAccumulatedAngleConstraint_setAccumulatedHingeAngle,
	SoftBody_Cluster_getImass = CLIB.btSoftBody_Cluster_getImass,
	SoftBody_Cluster_getFramexform = CLIB.btSoftBody_Cluster_getFramexform,
	AlignedObjectArray_btSoftBody_Node_push_back = CLIB.btAlignedObjectArray_btSoftBody_Node_push_back,
	CollisionWorld_ConvexResultCallback_getCollisionFilterGroup = CLIB.btCollisionWorld_ConvexResultCallback_getCollisionFilterGroup,
	ConeTwistConstraint_enableMotor = CLIB.btConeTwistConstraint_enableMotor,
	GImpactShapeInterface_getLocalBox = CLIB.btGImpactShapeInterface_getLocalBox,
	SoftBody_appendAngularJoint4 = CLIB.btSoftBody_appendAngularJoint4,
	ConvexConvexAlgorithm_CreateFunc_setMinimumPointsPerturbationThreshold = CLIB.btConvexConvexAlgorithm_CreateFunc_setMinimumPointsPerturbationThreshold,
	RigidBody_btRigidBodyConstructionInfo_setAdditionalAngularDampingThresholdSqr = CLIB.btRigidBody_btRigidBodyConstructionInfo_setAdditionalAngularDampingThresholdSqr,
	MultiBodyFixedConstraint_getFrameInA = CLIB.btMultiBodyFixedConstraint_getFrameInA,
	SoftBody_Cluster_getCollide = CLIB.btSoftBody_Cluster_getCollide,
	GImpactMeshShapePart_getVertex = CLIB.btGImpactMeshShapePart_getVertex,
	ConstraintSolver_prepareSolve = CLIB.btConstraintSolver_prepareSolve,
	GImpactCompoundShape_addChildShape = CLIB.btGImpactCompoundShape_addChildShape,
	SoftBody_Cluster_getClusterIndex = CLIB.btSoftBody_Cluster_getClusterIndex,
	Convex2dConvex2dAlgorithm_setLowLevelOfDetail = CLIB.btConvex2dConvex2dAlgorithm_setLowLevelOfDetail,
	SliderConstraint_setRestitutionDirAng = CLIB.btSliderConstraint_setRestitutionDirAng,
	ConeTwistConstraint_setAngularOnly = CLIB.btConeTwistConstraint_setAngularOnly,
	SoftBody_Cluster_getAdamping = CLIB.btSoftBody_Cluster_getAdamping,
	MaterialProperties_getNumTriangles = CLIB.btMaterialProperties_getNumTriangles,
	TranslationalLimitMotor_getDamping = CLIB.btTranslationalLimitMotor_getDamping,
	SoftBody_CJoint_setLife = CLIB.btSoftBody_CJoint_setLife,
	MultiBody_getKineticEnergy = CLIB.btMultiBody_getKineticEnergy,
	SoftBody_CJoint_setFriction = CLIB.btSoftBody_CJoint_setFriction,
	SoftBody_CJoint_getNormal = CLIB.btSoftBody_CJoint_getNormal,
	BroadphaseProxy_isInfinite = CLIB.btBroadphaseProxy_isInfinite,
	MultiBodyJointMotor_setVelocityTarget = CLIB.btMultiBodyJointMotor_setVelocityTarget,
	MultibodyLink_getJointName = CLIB.btMultibodyLink_getJointName,
	SliderConstraint_getPoweredLinMotor = CLIB.btSliderConstraint_getPoweredLinMotor,
	SoftBody_Body_velocity = CLIB.btSoftBody_Body_velocity,
	SoftBody_Body_setSoft = CLIB.btSoftBody_Body_setSoft,
	SoftBody_Body_setRigid = CLIB.btSoftBody_Body_setRigid,
	MultiBody_getUserIndex = CLIB.btMultiBody_getUserIndex,
	CollisionWorld_getPairCache = CLIB.btCollisionWorld_getPairCache,
	SoftBody_Body_setCollisionObject = CLIB.btSoftBody_Body_setCollisionObject,
	TransformUtil_integrateTransform = CLIB.btTransformUtil_integrateTransform,
	SimulationIslandManager_storeIslandActivationState = CLIB.btSimulationIslandManager_storeIslandActivationState,
	SoftBody_Body_invMass = CLIB.btSoftBody_Body_invMass,
	SoftBody_Body_getCollisionObject = CLIB.btSoftBody_Body_getCollisionObject,
	SoftBody_Body_applyVImpulse = CLIB.btSoftBody_Body_applyVImpulse,
	TransformUtil_calculateDiffAxisAngle = CLIB.btTransformUtil_calculateDiffAxisAngle,
	SoftBody_Body_applyImpulse = CLIB.btSoftBody_Body_applyImpulse,
	ConeTwistConstraint_calcAngleInfo2 = CLIB.btConeTwistConstraint_calcAngleInfo2,
	SoftBody_Body_applyDImpulse = CLIB.btSoftBody_Body_applyDImpulse,
	SoftBody_Body_applyDCImpulse = CLIB.btSoftBody_Body_applyDCImpulse,
	SoftBody_Body_applyDAImpulse = CLIB.btSoftBody_Body_applyDAImpulse,
	SoftBody_Cluster_getNdimpulses = CLIB.btSoftBody_Cluster_getNdimpulses,
	TranslationalLimitMotor2_getServoMotor = CLIB.btTranslationalLimitMotor2_getServoMotor,
	SoftBody_Body_new3 = CLIB.btSoftBody_Body_new3,
	SoftBody_Body_new2 = CLIB.btSoftBody_Body_new2,
	MultiBodyConstraint_finalizeMultiDof = CLIB.btMultiBodyConstraint_finalizeMultiDof,
	SoftBody_Body_new = CLIB.btSoftBody_Body_new,
	RigidBody_clearForces = CLIB.btRigidBody_clearForces,
	SoftBody_Anchor_setNode = CLIB.btSoftBody_Anchor_setNode,
	CollisionWorld_ConvexResultCallback_delete = CLIB.btCollisionWorld_ConvexResultCallback_delete,
	SoftBody_Anchor_setLocal = CLIB.btSoftBody_Anchor_setLocal,
	SoftBody_Anchor_setInfluence = CLIB.btSoftBody_Anchor_setInfluence,
	SoftBody_Joint_getDrift = CLIB.btSoftBody_Joint_getDrift,
	SoftBody_Anchor_setC1 = CLIB.btSoftBody_Anchor_setC1,
	DbvtBroadphase_getReleasepaircache = CLIB.btDbvtBroadphase_getReleasepaircache,
	SoftBody_Anchor_setC0 = CLIB.btSoftBody_Anchor_setC0,
	TranslationalLimitMotor2_testLimitValue = CLIB.btTranslationalLimitMotor2_testLimitValue,
	PersistentManifold_getCompanionIdA = CLIB.btPersistentManifold_getCompanionIdA,
	SoftBody_Anchor_setBody = CLIB.btSoftBody_Anchor_setBody,
	SoftBody_Anchor_getC2 = CLIB.btSoftBody_Anchor_getC2,
	RotationalLimitMotor_getStopCFM = CLIB.btRotationalLimitMotor_getStopCFM,
	SoftBody_Anchor_getC1 = CLIB.btSoftBody_Anchor_getC1,
	SoftBody_Anchor_getC0 = CLIB.btSoftBody_Anchor_getC0,
	NNCGConstraintSolver_new = CLIB.btNNCGConstraintSolver_new,
	SoftBody_AJoint_setIcontrol = CLIB.btSoftBody_AJoint_setIcontrol,
	SoftBody_AJoint_getIcontrol = CLIB.btSoftBody_AJoint_getIcontrol,
	Dbvt_getLkhd = CLIB.btDbvt_getLkhd,
	SoftBody_AJoint_getAxis = CLIB.btSoftBody_AJoint_getAxis,
	GImpactBvh_isLeafNode = CLIB.btGImpactBvh_isLeafNode,
	Dbvt_sStkCLN_setNode = CLIB.btDbvt_sStkCLN_setNode,
	SoftBody_AJoint_Specs_setIcontrol = CLIB.btSoftBody_AJoint_Specs_setIcontrol,
	GImpactShapeInterface_unlockChildShapes = CLIB.btGImpactShapeInterface_unlockChildShapes,
	SoftBody_AJoint_Specs_setAxis = CLIB.btSoftBody_AJoint_Specs_setAxis,
	SoftBody_addForce = CLIB.btSoftBody_addForce,
	Dbvt_IClone_new = CLIB.btDbvt_IClone_new,
	GImpactQuantizedBvh_get_node_pointer = CLIB.btGImpactQuantizedBvh_get_node_pointer,
	AngularLimit_getRelaxationFactor = CLIB.btAngularLimit_getRelaxationFactor,
	MultiBody_fillContactJacobianMultiDof = CLIB.btMultiBody_fillContactJacobianMultiDof,
	SoftBody_AJoint_IControl_Prepare = CLIB.btSoftBody_AJoint_IControl_Prepare,
	SoftBody_AJoint_IControlWrapper_setWrapperData = CLIB.btSoftBody_AJoint_IControlWrapper_setWrapperData,
	TranslationalLimitMotor2_getMotorCFM = CLIB.btTranslationalLimitMotor2_getMotorCFM,
	ManifoldPoint_new = CLIB.btManifoldPoint_new,
	SoftBody_AJoint_IControlWrapper_new = CLIB.btSoftBody_AJoint_IControlWrapper_new,
	HingeConstraint_setMaxMotorImpulse = CLIB.btHingeConstraint_setMaxMotorImpulse,
	SoftBodyWorldInfo_setWater_offset = CLIB.btSoftBodyWorldInfo_setWater_offset,
	SoftBodyWorldInfo_setWater_normal = CLIB.btSoftBodyWorldInfo_setWater_normal,
	SoftBodyWorldInfo_setWater_density = CLIB.btSoftBodyWorldInfo_setWater_density,
	CollisionObject_getCcdSweptSphereRadius = CLIB.btCollisionObject_getCcdSweptSphereRadius,
	AABB_getMin = CLIB.btAABB_getMin,
	SoftBodyWorldInfo_setMaxDisplacement = CLIB.btSoftBodyWorldInfo_setMaxDisplacement,
	SoftBody_getWindVelocity = CLIB.btSoftBody_getWindVelocity,
	SoftBodyWorldInfo_setDispatcher = CLIB.btSoftBodyWorldInfo_setDispatcher,
	SoftBodyWorldInfo_setBroadphase = CLIB.btSoftBodyWorldInfo_setBroadphase,
	TriangleMeshShape_getMeshInterface = CLIB.btTriangleMeshShape_getMeshInterface,
	RigidBody_btRigidBodyConstructionInfo_getFriction = CLIB.btRigidBody_btRigidBodyConstructionInfo_getFriction,
	RigidBody_getFrictionSolverType = CLIB.btRigidBody_getFrictionSolverType,
	SoftBodyWorldInfo_getWater_normal = CLIB.btSoftBodyWorldInfo_getWater_normal,
	MultiBodySolverConstraint_setFrictionIndex = CLIB.btMultiBodySolverConstraint_setFrictionIndex,
	SoftBodyWorldInfo_getMaxDisplacement = CLIB.btSoftBodyWorldInfo_getMaxDisplacement,
	HingeConstraint_setUseReferenceFrameA = CLIB.btHingeConstraint_setUseReferenceFrameA,
	ConvexConcaveCollisionAlgorithm_clearCache = CLIB.btConvexConcaveCollisionAlgorithm_clearCache,
	SoftBodyWorldInfo_getGravity = CLIB.btSoftBodyWorldInfo_getGravity,
	Generic6DofSpring2Constraint_getTranslationalLimitMotor = CLIB.btGeneric6DofSpring2Constraint_getTranslationalLimitMotor,
	SoftBodyWorldInfo_getDispatcher = CLIB.btSoftBodyWorldInfo_getDispatcher,
	BvhTriangleMeshShape_getTriangleInfoMap = CLIB.btBvhTriangleMeshShape_getTriangleInfoMap,
	CompoundShape_getChildList = CLIB.btCompoundShape_getChildList,
	ContactSolverInfoData_getNumIterations = CLIB.btContactSolverInfoData_getNumIterations,
	BvhTriangleMeshShape_buildOptimizedBvh = CLIB.btBvhTriangleMeshShape_buildOptimizedBvh,
	AxisSweep3_getOverlappingPairUserCallback = CLIB.btAxisSweep3_getOverlappingPairUserCallback,
	Generic6DofSpringConstraint_new2 = CLIB.btGeneric6DofSpringConstraint_new2,
	SoftBodyWorldInfo_getAir_density = CLIB.btSoftBodyWorldInfo_getAir_density,
	MultiBodySolverConstraint_getJacDiagABInv = CLIB.btMultiBodySolverConstraint_getJacDiagABInv,
	ManifoldPoint_setCombinedContactStiffness1 = CLIB.btManifoldPoint_setCombinedContactStiffness1,
	SliderConstraint_testLinLimits = CLIB.btSliderConstraint_testLinLimits,
	MultiBodySolverConstraint_getMultiBodyB = CLIB.btMultiBodySolverConstraint_getMultiBodyB,
	SoftBody_getBUpdateRtCst = CLIB.btSoftBody_getBUpdateRtCst,
	EmptyShape_new = CLIB.btEmptyShape_new,
	Triangle_getVertex0 = CLIB.btTriangle_getVertex0,
	SliderConstraint_setUpperAngLimit = CLIB.btSliderConstraint_setUpperAngLimit,
	SliderConstraint_setTargetLinMotorVelocity = CLIB.btSliderConstraint_setTargetLinMotorVelocity,
	Hinge2Constraint_getAngle2 = CLIB.btHinge2Constraint_getAngle2,
	SliderConstraint_setTargetAngMotorVelocity = CLIB.btSliderConstraint_setTargetAngMotorVelocity,
	SoftBody_Node_getN = CLIB.btSoftBody_Node_getN,
	SliderConstraint_getDampingDirLin = CLIB.btSliderConstraint_getDampingDirLin,
	SliderConstraint_setSoftnessLimAng = CLIB.btSliderConstraint_setSoftnessLimAng,
	SliderConstraint_setSoftnessDirAng = CLIB.btSliderConstraint_setSoftnessDirAng,
	SliderConstraint_setRestitutionOrthoLin = CLIB.btSliderConstraint_setRestitutionOrthoLin,
	RigidBody_btRigidBodyConstructionInfo_setLinearSleepingThreshold = CLIB.btRigidBody_btRigidBodyConstructionInfo_setLinearSleepingThreshold,
	DbvtNode_getVolume = CLIB.btDbvtNode_getVolume,
	BroadphasePair_setAlgorithm = CLIB.btBroadphasePair_setAlgorithm,
	OverlappingPairCallback_removeOverlappingPairsContainingProxy = CLIB.btOverlappingPairCallback_removeOverlappingPairsContainingProxy,
	JointFeedback_setAppliedForceBodyB = CLIB.btJointFeedback_setAppliedForceBodyB,
	SoftBody_appendLinearJoint4 = CLIB.btSoftBody_appendLinearJoint4,
	TranslationalLimitMotor_delete = CLIB.btTranslationalLimitMotor_delete,
	CollisionShape_calculateLocalInertia = CLIB.btCollisionShape_calculateLocalInertia,
	ConvexConvexAlgorithm_CreateFunc_new = CLIB.btConvexConvexAlgorithm_CreateFunc_new,
	SliderConstraint_setPoweredLinMotor = CLIB.btSliderConstraint_setPoweredLinMotor,
	MultiBodyDynamicsWorld_forwardKinematics = CLIB.btMultiBodyDynamicsWorld_forwardKinematics,
	OverlappingPairCache_findPair = CLIB.btOverlappingPairCache_findPair,
	SliderConstraint_setPoweredAngMotor = CLIB.btSliderConstraint_setPoweredAngMotor,
	SliderConstraint_setMaxLinMotorForce = CLIB.btSliderConstraint_setMaxLinMotorForce,
	TriangleIndexVertexMaterialArray_getLockedMaterialBase = CLIB.btTriangleIndexVertexMaterialArray_getLockedMaterialBase,
	SoftBody_Node_getIm = CLIB.btSoftBody_Node_getIm,
	OverlappingPairCallback_removeOverlappingPair = CLIB.btOverlappingPairCallback_removeOverlappingPair,
	SliderConstraint_setLowerAngLimit = CLIB.btSliderConstraint_setLowerAngLimit,
	DbvtBroadphase_setPrediction = CLIB.btDbvtBroadphase_setPrediction,
	ContactSolverInfoData_setSplitImpulsePenetrationThreshold = CLIB.btContactSolverInfoData_setSplitImpulsePenetrationThreshold,
	StorageResult_setNormalOnSurfaceB = CLIB.btStorageResult_setNormalOnSurfaceB,
	SliderConstraint_calculateTransforms = CLIB.btSliderConstraint_calculateTransforms,
	SimulationIslandManager_getSplitIslands = CLIB.btSimulationIslandManager_getSplitIslands,
	SliderConstraint_setDampingOrthoAng = CLIB.btSliderConstraint_setDampingOrthoAng,
	SliderConstraint_setDampingLimLin = CLIB.btSliderConstraint_setDampingLimLin,
	SliderConstraint_setDampingLimAng = CLIB.btSliderConstraint_setDampingLimAng,
	SoftBodyHelpers_DrawFaceTree = CLIB.btSoftBodyHelpers_DrawFaceTree,
	SliderConstraint_setDampingDirAng = CLIB.btSliderConstraint_setDampingDirAng,
	SliderConstraint_getUseFrameOffset = CLIB.btSliderConstraint_getUseFrameOffset,
	Generic6DofSpringConstraint_enableSpring = CLIB.btGeneric6DofSpringConstraint_enableSpring,
	MultibodyLink_getAppliedConstraintForce = CLIB.btMultibodyLink_getAppliedConstraintForce,
	CompoundShapeChild_setChildShapeType = CLIB.btCompoundShapeChild_setChildShapeType,
	ConvexHullShape_addPoint = CLIB.btConvexHullShape_addPoint,
	SliderConstraint_getUpperLinLimit = CLIB.btSliderConstraint_getUpperLinLimit,
	SliderConstraint_getUpperAngLimit = CLIB.btSliderConstraint_getUpperAngLimit,
	SliderConstraint_getTargetLinMotorVelocity = CLIB.btSliderConstraint_getTargetLinMotorVelocity,
	CollisionDispatcher_registerClosestPointsCreateFunc = CLIB.btCollisionDispatcher_registerClosestPointsCreateFunc,
	DbvtAabbMm_delete = CLIB.btDbvtAabbMm_delete,
	ManifoldPoint_getPartId1 = CLIB.btManifoldPoint_getPartId1,
	SliderConstraint_getSolveLinLimit = CLIB.btSliderConstraint_getSolveLinLimit,
	SliderConstraint_getSolveAngLimit = CLIB.btSliderConstraint_getSolveAngLimit,
	SliderConstraint_getSoftnessLimLin = CLIB.btSliderConstraint_getSoftnessLimLin,
	SliderConstraint_getTargetAngMotorVelocity = CLIB.btSliderConstraint_getTargetAngMotorVelocity,
	ContactSolverInfoData_setRestitution = CLIB.btContactSolverInfoData_setRestitution,
	AlignedObjectArray_btSoftBody_Node_index_of = CLIB.btAlignedObjectArray_btSoftBody_Node_index_of,
	SliderConstraint_getRestitutionOrthoAng = CLIB.btSliderConstraint_getRestitutionOrthoAng,
	MultibodyLink_setLinkName = CLIB.btMultibodyLink_setLinkName,
	GImpactBvh_boxQuery = CLIB.btGImpactBvh_boxQuery,
	AlignedObjectArray_btVector3_at = CLIB.btAlignedObjectArray_btVector3_at,
	MultiBodySolverConstraint_getFriction = CLIB.btMultiBodySolverConstraint_getFriction,
	CompoundShape_getNumChildShapes = CLIB.btCompoundShape_getNumChildShapes,
	SliderConstraint_getRestitutionDirAng = CLIB.btSliderConstraint_getRestitutionDirAng,
	SoftBody_CJoint_getFriction = CLIB.btSoftBody_CJoint_getFriction,
	SliderConstraint_getPoweredAngMotor = CLIB.btSliderConstraint_getPoweredAngMotor,
	SliderConstraint_getMaxLinMotorForce = CLIB.btSliderConstraint_getMaxLinMotorForce,
	SliderConstraint_getLowerLinLimit = CLIB.btSliderConstraint_getLowerLinLimit,
	SliderConstraint_getLowerAngLimit = CLIB.btSliderConstraint_getLowerAngLimit,
	TranslationalLimitMotor_getUpperLimit = CLIB.btTranslationalLimitMotor_getUpperLimit,
	Box2dShape_getVertexCount = CLIB.btBox2dShape_getVertexCount,
	HingeConstraint_setLimit = CLIB.btHingeConstraint_setLimit,
	SliderConstraint_getLinDepth = CLIB.btSliderConstraint_getLinDepth,
	SliderConstraint_getInfo2NonVirtual = CLIB.btSliderConstraint_getInfo2NonVirtual,
	PairSet_delete = CLIB.btPairSet_delete,
	DbvtNode_getChilds = CLIB.btDbvtNode_getChilds,
	SliderConstraint_getInfo1NonVirtual = CLIB.btSliderConstraint_getInfo1NonVirtual,
	BvhTree_isLeafNode = CLIB.btBvhTree_isLeafNode,
	DbvtAabbMm_Mins = CLIB.btDbvtAabbMm_Mins,
	DbvtBroadphase_collide = CLIB.btDbvtBroadphase_collide,
	SliderConstraint_getDampingOrthoAng = CLIB.btSliderConstraint_getDampingOrthoAng,
	SliderConstraint_setSoftnessOrthoAng = CLIB.btSliderConstraint_setSoftnessOrthoAng,
	ManifoldPoint_getLocalPointA = CLIB.btManifoldPoint_getLocalPointA,
	SliderConstraint_getDampingDirAng = CLIB.btSliderConstraint_getDampingDirAng,
	SubSimplexClosestResult_delete = CLIB.btSubSimplexClosestResult_delete,
	SliderConstraint_getAngDepth = CLIB.btSliderConstraint_getAngDepth,
	Generic6DofSpring2Constraint_getRelativePivotPosition = CLIB.btGeneric6DofSpring2Constraint_getRelativePivotPosition,
	SliderConstraint_getAncorInA = CLIB.btSliderConstraint_getAncorInA,
	DbvtProxy_setStage = CLIB.btDbvtProxy_setStage,
	ContactSolverInfoData_getMaxErrorReduction = CLIB.btContactSolverInfoData_getMaxErrorReduction,
	SimulationIslandManager_delete = CLIB.btSimulationIslandManager_delete,
	SliderConstraint_new2 = CLIB.btSliderConstraint_new2,
	SimulationIslandManager_setSplitIslands = CLIB.btSimulationIslandManager_setSplitIslands,
	Box2dShape_getCentroid = CLIB.btBox2dShape_getCentroid,
	SliderConstraint_setDampingOrthoLin = CLIB.btSliderConstraint_setDampingOrthoLin,
	SimulationIslandManager_findUnions = CLIB.btSimulationIslandManager_findUnions,
	CollisionObject_forceActivationState = CLIB.btCollisionObject_forceActivationState,
	MotionState_getWorldTransform = CLIB.btMotionState_getWorldTransform,
	RigidBody_getAngularDamping = CLIB.btRigidBody_getAngularDamping,
	SimulationIslandManager_buildAndProcessIslands = CLIB.btSimulationIslandManager_buildAndProcessIslands,
	PrimitiveTriangle_buildTriPlane = CLIB.btPrimitiveTriangle_buildTriPlane,
	SimulationIslandManager_IslandCallback_delete = CLIB.btSimulationIslandManager_IslandCallback_delete,
	SimulationIslandManager_IslandCallback_processIsland = CLIB.btSimulationIslandManager_IslandCallback_processIsland,
	ShapeHull_delete = CLIB.btShapeHull_delete,
	RigidBody_saveKinematicState = CLIB.btRigidBody_saveKinematicState,
	PolyhedralConvexShape_getNumVertices = CLIB.btPolyhedralConvexShape_getNumVertices,
	TranslationalLimitMotor2_getSpringStiffnessLimited = CLIB.btTranslationalLimitMotor2_getSpringStiffnessLimited,
	ShapeHull_numIndices = CLIB.btShapeHull_numIndices,
	ShapeHull_getVertexPointer = CLIB.btShapeHull_getVertexPointer,
	SoftRigidDynamicsWorld_addSoftBody3 = CLIB.btSoftRigidDynamicsWorld_addSoftBody3,
	CollisionObject_checkCollideWithOverride = CLIB.btCollisionObject_checkCollideWithOverride,
	MultiBody_addBaseConstraintTorque = CLIB.btMultiBody_addBaseConstraintTorque,
	MotionState_delete = CLIB.btMotionState_delete,
	Generic6DofConstraint_setAngularLowerLimit = CLIB.btGeneric6DofConstraint_setAngularLowerLimit,
	AlignedObjectArray_btSoftBodyPtr_push_back = CLIB.btAlignedObjectArray_btSoftBodyPtr_push_back,
	ShapeHull_new = CLIB.btShapeHull_new,
	UsageBitfield_getUsedVertexA = CLIB.btUsageBitfield_getUsedVertexA,
	CollisionWorld_ContactResultCallback_getCollisionFilterGroup = CLIB.btCollisionWorld_ContactResultCallback_getCollisionFilterGroup,
	DefaultSerializer_writeHeader = CLIB.btDefaultSerializer_writeHeader,
	SliderConstraint_getLinearPos = CLIB.btSliderConstraint_getLinearPos,
	DefaultSerializer_new = CLIB.btDefaultSerializer_new,
	SoftBody_Config_getTimescale = CLIB.btSoftBody_Config_getTimescale,
	TranslationalLimitMotor2_setUpperLimit = CLIB.btTranslationalLimitMotor2_setUpperLimit,
	Chunk_setNumber = CLIB.btChunk_setNumber,
	Chunk_setDna_nr = CLIB.btChunk_setDna_nr,
	HashedOverlappingPairCache_GetCount = CLIB.btHashedOverlappingPairCache_GetCount,
	Chunk_getOldPtr = CLIB.btChunk_getOldPtr,
	DbvtBroadphase_getNeedcleanup = CLIB.btDbvtBroadphase_getNeedcleanup,
	Chunk_getDna_nr = CLIB.btChunk_getDna_nr,
	BoxBoxDetector_new = CLIB.btBoxBoxDetector_new,
	Chunk_getChunkCode = CLIB.btChunk_getChunkCode,
	SequentialImpulseConstraintSolver_setRandSeed = CLIB.btSequentialImpulseConstraintSolver_setRandSeed,
	SequentialImpulseConstraintSolver_getRandSeed = CLIB.btSequentialImpulseConstraintSolver_getRandSeed,
	CollisionObjectWrapper_setIndex = CLIB.btCollisionObjectWrapper_setIndex,
	CollisionObject_getRollingFriction = CLIB.btCollisionObject_getRollingFriction,
	GImpactQuantizedBvh_getGlobalBox = CLIB.btGImpactQuantizedBvh_getGlobalBox,
	SequentialImpulseConstraintSolver_new = CLIB.btSequentialImpulseConstraintSolver_new,
	RigidBody_btRigidBodyConstructionInfo_getRollingFriction = CLIB.btRigidBody_btRigidBodyConstructionInfo_getRollingFriction,
	RigidBody_updateInertiaTensor = CLIB.btRigidBody_updateInertiaTensor,
	DispatcherInfo_setEnableSatConvex = CLIB.btDispatcherInfo_setEnableSatConvex,
	RigidBody_updateDeactivation = CLIB.btRigidBody_updateDeactivation,
	CollisionWorld_LocalShapeInfo_getShapePart = CLIB.btCollisionWorld_LocalShapeInfo_getShapePart,
	IndexedMesh_getVertexStride = CLIB.btIndexedMesh_getVertexStride,
	RigidBody_setSleepingThresholds = CLIB.btRigidBody_setSleepingThresholds,
	RotationalLimitMotor_getHiLimit = CLIB.btRotationalLimitMotor_getHiLimit,
	ConeTwistConstraint_getFrameOffsetB = CLIB.btConeTwistConstraint_getFrameOffsetB,
	RigidBody_setLinearFactor = CLIB.btRigidBody_setLinearFactor,
	DbvtBroadphase_new = CLIB.btDbvtBroadphase_new,
	DbvtNode_getData = CLIB.btDbvtNode_getData,
	RigidBody_setInvInertiaDiagLocal = CLIB.btRigidBody_setInvInertiaDiagLocal,
	RigidBody_setFrictionSolverType = CLIB.btRigidBody_setFrictionSolverType,
	RigidBody_setFlags = CLIB.btRigidBody_setFlags,
	RigidBody_setContactSolverType = CLIB.btRigidBody_setContactSolverType,
	CollisionObject_hasContactResponse = CLIB.btCollisionObject_hasContactResponse,
	CollisionWorld_ClosestConvexResultCallback_getHitNormalWorld = CLIB.btCollisionWorld_ClosestConvexResultCallback_getHitNormalWorld,
	RigidBody_setCenterOfMassTransform = CLIB.btRigidBody_setCenterOfMassTransform,
	DefaultCollisionConfiguration_getClosestPointsAlgorithmCreateFunc = CLIB.btDefaultCollisionConfiguration_getClosestPointsAlgorithmCreateFunc,
	ManifoldPoint_getContactPointFlags = CLIB.btManifoldPoint_getContactPointFlags,
	MultiBody_clearForcesAndTorques = CLIB.btMultiBody_clearForcesAndTorques,
	ManifoldResult_calculateCombinedRestitution = CLIB.btManifoldResult_calculateCombinedRestitution,
	ConvexPointCloudShape_setPoints2 = CLIB.btConvexPointCloudShape_setPoints2,
	RigidBody_isInWorld = CLIB.btRigidBody_isInWorld,
	DbvtBroadphase_getVelocityPrediction = CLIB.btDbvtBroadphase_getVelocityPrediction,
	ConvexConcaveCollisionAlgorithm_new = CLIB.btConvexConcaveCollisionAlgorithm_new,
	RigidBody_integrateVelocities = CLIB.btRigidBody_integrateVelocities,
	RigidBody_getTotalForce = CLIB.btRigidBody_getTotalForce,
	PersistentManifold_getContactProcessingThreshold = CLIB.btPersistentManifold_getContactProcessingThreshold,
	MaterialProperties_getNumMaterials = CLIB.btMaterialProperties_getNumMaterials,
	RotationalLimitMotor_delete = CLIB.btRotationalLimitMotor_delete,
	Dbvt_sStkNP_setMask = CLIB.btDbvt_sStkNP_setMask,
	MultiBody_getWorldToBaseRot = CLIB.btMultiBody_getWorldToBaseRot,
	AlignedObjectArray_btSoftBodyPtr_at = CLIB.btAlignedObjectArray_btSoftBodyPtr_at,
	BroadphaseProxy_getAabbMin = CLIB.btBroadphaseProxy_getAabbMin,
	CollisionShape_getBoundingSphere = CLIB.btCollisionShape_getBoundingSphere,
	RigidBody_getInvMass = CLIB.btRigidBody_getInvMass,
	GImpactMeshShapePart_TrimeshPrimitiveManager_get_indices = CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_get_indices,
	Generic6DofSpring2Constraint_getRotationOrder = CLIB.btGeneric6DofSpring2Constraint_getRotationOrder,
	RigidBody_getInvInertiaDiagLocal = CLIB.btRigidBody_getInvInertiaDiagLocal,
	Point2PointConstraint_getPivotInA = CLIB.btPoint2PointConstraint_getPivotInA,
	RigidBody_getConstraintRef = CLIB.btRigidBody_getConstraintRef,
	GImpactCollisionAlgorithm_setFace1 = CLIB.btGImpactCollisionAlgorithm_setFace1,
	RigidBody_getBroadphaseProxy = CLIB.btRigidBody_getBroadphaseProxy,
	CollisionShape_getShapeType = CLIB.btCollisionShape_getShapeType,
	MultiBodyConstraint_isUnilateral = CLIB.btMultiBodyConstraint_isUnilateral,
	TranslationalLimitMotor2_getStopCFM = CLIB.btTranslationalLimitMotor2_getStopCFM,
	ConvexConvexAlgorithm_setLowLevelOfDetail = CLIB.btConvexConvexAlgorithm_setLowLevelOfDetail,
	RigidBody_getAngularSleepingThreshold = CLIB.btRigidBody_getAngularSleepingThreshold,
	TranslationalLimitMotor_setNormalCFM = CLIB.btTranslationalLimitMotor_setNormalCFM,
	RigidBody_applyTorque = CLIB.btRigidBody_applyTorque,
	RigidBody_applyImpulse = CLIB.btRigidBody_applyImpulse,
	ConvexSeparatingDistanceUtil_new = CLIB.btConvexSeparatingDistanceUtil_new,
	CollisionDispatcher_getNearCallback = CLIB.btCollisionDispatcher_getNearCallback,
	BroadphaseAabbCallback_delete = CLIB.btBroadphaseAabbCallback_delete,
	ConvexCast_CastResult_setHitPoint = CLIB.btConvexCast_CastResult_setHitPoint,
	MultiBody_setBaseOmega = CLIB.btMultiBody_setBaseOmega,
	MultibodyLink_getAxes = CLIB.btMultibodyLink_getAxes,
	ManifoldPoint_setLocalPointA = CLIB.btManifoldPoint_setLocalPointA,
	RigidBody_applyForce = CLIB.btRigidBody_applyForce,
	RigidBody_applyDamping = CLIB.btRigidBody_applyDamping,
	SoftRigidDynamicsWorld_addSoftBody = CLIB.btSoftRigidDynamicsWorld_addSoftBody,
	RigidBody_addConstraintRef = CLIB.btRigidBody_addConstraintRef,
	SoftBody_applyForces = CLIB.btSoftBody_applyForces,
	Convex2dConvex2dAlgorithm_CreateFunc_setPdSolver = CLIB.btConvex2dConvex2dAlgorithm_CreateFunc_setPdSolver,
	RigidBody_btRigidBodyConstructionInfo_setRollingFriction = CLIB.btRigidBody_btRigidBodyConstructionInfo_setRollingFriction,
	Convex2dConvex2dAlgorithm_CreateFunc_setSimplexSolver = CLIB.btConvex2dConvex2dAlgorithm_CreateFunc_setSimplexSolver,
	RigidBody_btRigidBodyConstructionInfo_setRestitution = CLIB.btRigidBody_btRigidBodyConstructionInfo_setRestitution,
	RigidBody_btRigidBodyConstructionInfo_setMass = CLIB.btRigidBody_btRigidBodyConstructionInfo_setMass,
	Dbvt_sStkNP_delete = CLIB.btDbvt_sStkNP_delete,
	CollisionWorld_LocalRayResult_delete = CLIB.btCollisionWorld_LocalRayResult_delete,
	CollisionWorld_LocalShapeInfo_setTriangleIndex = CLIB.btCollisionWorld_LocalShapeInfo_setTriangleIndex,
	SliderConstraint_setRestitutionOrthoAng = CLIB.btSliderConstraint_setRestitutionOrthoAng,
	RigidBody_btRigidBodyConstructionInfo_setLinearDamping = CLIB.btRigidBody_btRigidBodyConstructionInfo_setLinearDamping,
	RotationalLimitMotor_getAccumulatedImpulse = CLIB.btRotationalLimitMotor_getAccumulatedImpulse,
	BroadphaseRayCallback_getSigns = CLIB.btBroadphaseRayCallback_getSigns,
	RigidBody_btRigidBodyConstructionInfo_setFriction = CLIB.btRigidBody_btRigidBodyConstructionInfo_setFriction,
	QuantizedBvh_deSerializeDouble = CLIB.btQuantizedBvh_deSerializeDouble,
	SoftBody_clusterImpulse = CLIB.btSoftBody_clusterImpulse,
	RigidBody_btRigidBodyConstructionInfo_setAngularSleepingThreshold = CLIB.btRigidBody_btRigidBodyConstructionInfo_setAngularSleepingThreshold,
	GImpactQuantizedBvh_delete = CLIB.btGImpactQuantizedBvh_delete,
	MultiBodyConstraint_getMultiBodyB = CLIB.btMultiBodyConstraint_getMultiBodyB,
	RigidBody_btRigidBodyConstructionInfo_setAngularDamping = CLIB.btRigidBody_btRigidBodyConstructionInfo_setAngularDamping,
	MultiBodyDynamicsWorld_debugDrawMultiBodyConstraint = CLIB.btMultiBodyDynamicsWorld_debugDrawMultiBodyConstraint,
	ConvexHullShape_getUnscaledPoints = CLIB.btConvexHullShape_getUnscaledPoints,
	MultiBody_getJointTorque = CLIB.btMultiBody_getJointTorque,
	RigidBody_btRigidBodyConstructionInfo_setAdditionalDampingFactor = CLIB.btRigidBody_btRigidBodyConstructionInfo_setAdditionalDampingFactor,
	RigidBody_btRigidBodyConstructionInfo_setAdditionalDamping = CLIB.btRigidBody_btRigidBodyConstructionInfo_setAdditionalDamping,
	AlignedObjectArray_btCollisionObjectPtr_push_back = CLIB.btAlignedObjectArray_btCollisionObjectPtr_push_back,
	SoftBody_Cluster_getContainsAnchor = CLIB.btSoftBody_Cluster_getContainsAnchor,
	RigidBody_btRigidBodyConstructionInfo_setAdditionalAngularDampingFactor = CLIB.btRigidBody_btRigidBodyConstructionInfo_setAdditionalAngularDampingFactor,
	RigidBody_btRigidBodyConstructionInfo_getStartWorldTransform = CLIB.btRigidBody_btRigidBodyConstructionInfo_getStartWorldTransform,
	ConeShape_new = CLIB.btConeShape_new,
	BvhTriangleMeshShape_performConvexcast = CLIB.btBvhTriangleMeshShape_performConvexcast,
	ConcaveShape_processAllTriangles = CLIB.btConcaveShape_processAllTriangles,
	CompoundShapeChild_delete = CLIB.btCompoundShapeChild_delete,
	ScaledBvhTriangleMeshShape_getChildShape = CLIB.btScaledBvhTriangleMeshShape_getChildShape,
	RigidBody_btRigidBodyConstructionInfo_getRestitution = CLIB.btRigidBody_btRigidBodyConstructionInfo_getRestitution,
	GImpactMeshShapePart_TrimeshPrimitiveManager_getNumfaces = CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_getNumfaces,
	DbvtBroadphase_setCid = CLIB.btDbvtBroadphase_setCid,
	HingeConstraint_getInfo2NonVirtual = CLIB.btHingeConstraint_getInfo2NonVirtual,
	ConvexSeparatingDistanceUtil_delete = CLIB.btConvexSeparatingDistanceUtil_delete,
	RigidBody_btRigidBodyConstructionInfo_getMass = CLIB.btRigidBody_btRigidBodyConstructionInfo_getMass,
	SoftBodySolver_copyBackToSoftBodies = CLIB.btSoftBodySolver_copyBackToSoftBodies,
	MultiBody_setBaseMass = CLIB.btMultiBody_setBaseMass,
	OptimizedBvhNode_setEscapeIndex = CLIB.btOptimizedBvhNode_setEscapeIndex,
	RigidBody_btRigidBodyConstructionInfo_getLinearDamping = CLIB.btRigidBody_btRigidBodyConstructionInfo_getLinearDamping,
	RigidBody_btRigidBodyConstructionInfo_getCollisionShape = CLIB.btRigidBody_btRigidBodyConstructionInfo_getCollisionShape,
	RigidBody_btRigidBodyConstructionInfo_getAngularSleepingThreshold = CLIB.btRigidBody_btRigidBodyConstructionInfo_getAngularSleepingThreshold,
	HingeConstraint_getLimitRelaxationFactor = CLIB.btHingeConstraint_getLimitRelaxationFactor,
	RigidBody_btRigidBodyConstructionInfo_getAngularDamping = CLIB.btRigidBody_btRigidBodyConstructionInfo_getAngularDamping,
	GjkPairDetector_setFixContactNormalDirection = CLIB.btGjkPairDetector_setFixContactNormalDirection,
	MultiBody_setUserIndex2 = CLIB.btMultiBody_setUserIndex2,
	ManifoldResult_setBody1Wrap = CLIB.btManifoldResult_setBody1Wrap,
	MultiBodyDynamicsWorld_clearMultiBodyConstraintForces = CLIB.btMultiBodyDynamicsWorld_clearMultiBodyConstraintForces,
	DbvtAabbMm_Contain = CLIB.btDbvtAabbMm_Contain,
	Dbvt_clone2 = CLIB.btDbvt_clone2,
	PrimitiveTriangle_find_triangle_collision_clip_method = CLIB.btPrimitiveTriangle_find_triangle_collision_clip_method,
	GImpactShapeInterface_needsRetrieveTriangles = CLIB.btGImpactShapeInterface_needsRetrieveTriangles,
	CollisionWorld_computeOverlappingPairs = CLIB.btCollisionWorld_computeOverlappingPairs,
	CollisionDispatcher_getDispatcherFlags = CLIB.btCollisionDispatcher_getDispatcherFlags,
	RotationalLimitMotor_new = CLIB.btRotationalLimitMotor_new,
	MultiBodyJointMotor_setPositionTarget2 = CLIB.btMultiBodyJointMotor_setPositionTarget2,
	MultiBody_hasSelfCollision = CLIB.btMultiBody_hasSelfCollision,
	ManifoldResult_new2 = CLIB.btManifoldResult_new2,
	PolarDecomposition_maxIterations = CLIB.btPolarDecomposition_maxIterations,
	ConeTwistConstraint_updateRHS = CLIB.btConeTwistConstraint_updateRHS,
	Convex2dConvex2dAlgorithm_CreateFunc_setMinimumPointsPerturbationThreshold = CLIB.btConvex2dConvex2dAlgorithm_CreateFunc_setMinimumPointsPerturbationThreshold,
	TranslationalLimitMotor_setStopERP = CLIB.btTranslationalLimitMotor_setStopERP,
	Box2dShape_getVertices = CLIB.btBox2dShape_getVertices,
	MultibodyLink_setAxisTop = CLIB.btMultibodyLink_setAxisTop,
	Dbvt_setLkhd = CLIB.btDbvt_setLkhd,
	GImpactShapeInterface_setChildTransform = CLIB.btGImpactShapeInterface_setChildTransform,
	QuantizedBvhTree_getNodeCount = CLIB.btQuantizedBvhTree_getNodeCount,
	IndexedMesh_setVertexType = CLIB.btIndexedMesh_setVertexType,
	AlignedObjectArray_btPersistentManifoldPtr_push_back = CLIB.btAlignedObjectArray_btPersistentManifoldPtr_push_back,
	QuantizedBvh_unQuantize = CLIB.btQuantizedBvh_unQuantize,
	QuantizedBvh_setTraversalMode = CLIB.btQuantizedBvh_setTraversalMode,
	SoftBody_updateArea = CLIB.btSoftBody_updateArea,
	ActionInterface_updateAction = CLIB.btActionInterface_updateAction,
	ConvexCast_CastResult_setHitTransformB = CLIB.btConvexCast_CastResult_setHitTransformB,
	ManifoldPoint_setLifeTime = CLIB.btManifoldPoint_setLifeTime,
	MultibodyLink_getDofOffset = CLIB.btMultibodyLink_getDofOffset,
	QuantizedBvh_serialize2 = CLIB.btQuantizedBvh_serialize2,
	StorageResult_getNormalOnSurfaceB = CLIB.btStorageResult_getNormalOnSurfaceB,
	SoftBody_appendLinearJoint = CLIB.btSoftBody_appendLinearJoint,
	CollisionWorld_new = CLIB.btCollisionWorld_new,
	QuantizedBvh_reportBoxCastOverlappingNodex = CLIB.btQuantizedBvh_reportBoxCastOverlappingNodex,
	CompoundShape_removeChildShapeByIndex = CLIB.btCompoundShape_removeChildShapeByIndex,
	PersistentManifold_validContactDistance = CLIB.btPersistentManifold_validContactDistance,
	MultiBody_localFrameToWorld = CLIB.btMultiBody_localFrameToWorld,
	QuantizedBvh_reportAabbOverlappingNodex = CLIB.btQuantizedBvh_reportAabbOverlappingNodex,
	BroadphaseProxy_isConvex = CLIB.btBroadphaseProxy_isConvex,
	ManifoldPoint_setAppliedImpulse = CLIB.btManifoldPoint_setAppliedImpulse,
	QuantizedBvh_quantize = CLIB.btQuantizedBvh_quantize,
	Generic6DofSpringConstraint_getDamping = CLIB.btGeneric6DofSpringConstraint_getDamping,
	QuantizedBvh_getSubtreeInfoArray = CLIB.btQuantizedBvh_getSubtreeInfoArray,
	PersistentManifold_getNumContacts = CLIB.btPersistentManifold_getNumContacts,
	DiscreteCollisionDetectorInterface_ClosestPointInput_new = CLIB.btDiscreteCollisionDetectorInterface_ClosestPointInput_new,
	QuantizedBvh_getQuantizedNodeArray = CLIB.btQuantizedBvh_getQuantizedNodeArray,
	QuantizedBvh_getAlignmentSerializationPadding = CLIB.btQuantizedBvh_getAlignmentSerializationPadding,
	QuantizedBvh_deSerializeInPlace = CLIB.btQuantizedBvh_deSerializeInPlace,
	QuantizedBvh_buildInternal = CLIB.btQuantizedBvh_buildInternal,
	MultiBodyLinkCollider_setMultiBody = CLIB.btMultiBodyLinkCollider_setMultiBody,
	NodeOverlapCallback_processNode = CLIB.btNodeOverlapCallback_processNode,
	VoronoiSimplexSolver_getSimplexVectorW = CLIB.btVoronoiSimplexSolver_getSimplexVectorW,
	OptimizedBvhNode_setSubPart = CLIB.btOptimizedBvhNode_setSubPart,
	RigidBody_btRigidBodyConstructionInfo_getLinearSleepingThreshold = CLIB.btRigidBody_btRigidBodyConstructionInfo_getLinearSleepingThreshold,
	OptimizedBvhNode_setAabbMinOrg = CLIB.btOptimizedBvhNode_setAabbMinOrg,
	DefaultMotionState_new3 = CLIB.btDefaultMotionState_new3,
	OptimizedBvhNode_setAabbMaxOrg = CLIB.btOptimizedBvhNode_setAabbMaxOrg,
	OptimizedBvhNode_getEscapeIndex = CLIB.btOptimizedBvhNode_getEscapeIndex,
	OptimizedBvhNode_getAabbMinOrg = CLIB.btOptimizedBvhNode_getAabbMinOrg,
	OptimizedBvhNode_new = CLIB.btOptimizedBvhNode_new,
	QuantizedBvhNode_delete = CLIB.btQuantizedBvhNode_delete,
	QuantizedBvhNode_setEscapeIndexOrTriangleIndex = CLIB.btQuantizedBvhNode_setEscapeIndexOrTriangleIndex,
	QuantizedBvhNode_isLeafNode = CLIB.btQuantizedBvhNode_isLeafNode,
	QuantizedBvhNode_getTriangleIndex = CLIB.btQuantizedBvhNode_getTriangleIndex,
	AABB_new2 = CLIB.btAABB_new2,
	AngularLimit_getSoftness = CLIB.btAngularLimit_getSoftness,
	TriangleIndexVertexMaterialArray_getLockedReadOnlyMaterialBase = CLIB.btTriangleIndexVertexMaterialArray_getLockedReadOnlyMaterialBase,
	QuantizedBvhNode_getPartId = CLIB.btQuantizedBvhNode_getPartId,
	GImpactBvh_hasHierarchy = CLIB.btGImpactBvh_hasHierarchy,
	UnionFind_isRoot = CLIB.btUnionFind_isRoot,
	QuantizedBvhNode_new = CLIB.btQuantizedBvhNode_new,
	PolyhedralConvexShape_isInside = CLIB.btPolyhedralConvexShape_isInside,
	MultiBodySolverConstraint_setOverrideNumSolverIterations = CLIB.btMultiBodySolverConstraint_setOverrideNumSolverIterations,
	MultibodyLink_getAxisBottom = CLIB.btMultibodyLink_getAxisBottom,
	ManifoldPoint_setLocalPointB = CLIB.btManifoldPoint_setLocalPointB,
	DynamicsWorld_synchronizeMotionStates = CLIB.btDynamicsWorld_synchronizeMotionStates,
	MultiBody_setJointVelMultiDof = CLIB.btMultiBody_setJointVelMultiDof,
	SoftBody_Link_getC3 = CLIB.btSoftBody_Link_getC3,
	CollisionWorld_RayResultCallback_getClosestHitFraction = CLIB.btCollisionWorld_RayResultCallback_getClosestHitFraction,
	ShapeHull_numVertices = CLIB.btShapeHull_numVertices,
	PolyhedralConvexShape_getNumEdges = CLIB.btPolyhedralConvexShape_getNumEdges,
	ManifoldPoint_getLocalPointB = CLIB.btManifoldPoint_getLocalPointB,
	SoftBody_getTetraVertexData = CLIB.btSoftBody_getTetraVertexData,
	PolarDecomposition_delete = CLIB.btPolarDecomposition_delete,
	PolarDecomposition_decompose = CLIB.btPolarDecomposition_decompose,
	MultibodyLink_getJointPos = CLIB.btMultibodyLink_getJointPos,
	AlignedObjectArray_btSoftBody_Face_push_back = CLIB.btAlignedObjectArray_btSoftBody_Face_push_back,
	MultiBodyConstraint_getAppliedImpulse = CLIB.btMultiBodyConstraint_getAppliedImpulse,
	PointCollector_setHasResult = CLIB.btPointCollector_setHasResult,
	DiscreteDynamicsWorld_applyGravity = CLIB.btDiscreteDynamicsWorld_applyGravity,
	Dbvt_clear = CLIB.btDbvt_clear,
	BroadphasePair_new3 = CLIB.btBroadphasePair_new3,
	PointCollector_setDistance = CLIB.btPointCollector_setDistance,
	AlignedObjectArray_btSoftBodyPtr_size = CLIB.btAlignedObjectArray_btSoftBodyPtr_size,
	HingeConstraint_new = CLIB.btHingeConstraint_new,
	PointCollector_getNormalOnBInWorld = CLIB.btPointCollector_getNormalOnBInWorld,
	PointCollector_getHasResult = CLIB.btPointCollector_getHasResult,
	PointCollector_getDistance = CLIB.btPointCollector_getDistance,
	PointCollector_new = CLIB.btPointCollector_new,
	ConvexInternalShape_getLocalScalingNV = CLIB.btConvexInternalShape_getLocalScalingNV,
	Point2PointConstraint_updateRHS = CLIB.btPoint2PointConstraint_updateRHS,
	Generic6DofSpring2Constraint_getAngularLowerLimit = CLIB.btGeneric6DofSpring2Constraint_getAngularLowerLimit,
	Point2PointConstraint_setUseSolveConstraintObsolete = CLIB.btPoint2PointConstraint_setUseSolveConstraintObsolete,
	MultiBody_setJointPosMultiDof = CLIB.btMultiBody_setJointPosMultiDof,
	BroadphaseInterface_rayTest = CLIB.btBroadphaseInterface_rayTest,
	Point2PointConstraint_getPivotInB = CLIB.btPoint2PointConstraint_getPivotInB,
	RigidBody_getContactSolverType = CLIB.btRigidBody_getContactSolverType,
	JointFeedback_getAppliedTorqueBodyA = CLIB.btJointFeedback_getAppliedTorqueBodyA,
	Dbvt_sStkCLN_setParent = CLIB.btDbvt_sStkCLN_setParent,
	Point2PointConstraint_new = CLIB.btPoint2PointConstraint_new,
	ConstraintSetting_delete = CLIB.btConstraintSetting_delete,
	RotationalLimitMotor2_getMotorCFM = CLIB.btRotationalLimitMotor2_getMotorCFM,
	ConstraintSetting_setTau = CLIB.btConstraintSetting_setTau,
	MultiBody_calculateSerializeBufferSize = CLIB.btMultiBody_calculateSerializeBufferSize,
	NullPairCache_new = CLIB.btNullPairCache_new,
	TypedConstraint_getDbgDrawSize = CLIB.btTypedConstraint_getDbgDrawSize,
	MultiBodySolverConstraint_getCfm = CLIB.btMultiBodySolverConstraint_getCfm,
	ConstraintSetting_setDamping = CLIB.btConstraintSetting_setDamping,
	SubSimplexClosestResult_reset = CLIB.btSubSimplexClosestResult_reset,
	GhostObject_addOverlappingObjectInternal = CLIB.btGhostObject_addOverlappingObjectInternal,
	GearConstraint_getRatio = CLIB.btGearConstraint_getRatio,
	AlignedObjectArray_btSoftBody_Anchor_at = CLIB.btAlignedObjectArray_btSoftBody_Anchor_at,
	MaterialProperties_setTriangleMaterialStride = CLIB.btMaterialProperties_setTriangleMaterialStride,
	MultiBody_computeAccelerationsArticulatedBodyAlgorithmMultiDof = CLIB.btMultiBody_computeAccelerationsArticulatedBodyAlgorithmMultiDof,
	Generic6DofSpring2Constraint_setDamping = CLIB.btGeneric6DofSpring2Constraint_setDamping,
	HingeConstraint_enableAngularMotor = CLIB.btHingeConstraint_enableAngularMotor,
	GImpactQuantizedBvh_getNodeTriangle = CLIB.btGImpactQuantizedBvh_getNodeTriangle,
	PersistentManifold_setNumContacts = CLIB.btPersistentManifold_setNumContacts,
	PersistentManifold_setIndex1a = CLIB.btPersistentManifold_setIndex1a,
	Box2dBox2dCollisionAlgorithm_new2 = CLIB.btBox2dBox2dCollisionAlgorithm_new2,
	CollisionWorld_objectQuerySingleInternal = CLIB.btCollisionWorld_objectQuerySingleInternal,
	PersistentManifold_setContactProcessingThreshold = CLIB.btPersistentManifold_setContactProcessingThreshold,
	PersistentManifold_setContactBreakingThreshold = CLIB.btPersistentManifold_setContactBreakingThreshold,
	MultiSphereShape_new2 = CLIB.btMultiSphereShape_new2,
	TranslationalLimitMotor_getCurrentLimitError = CLIB.btTranslationalLimitMotor_getCurrentLimitError,
	PersistentManifold_setCompanionIdB = CLIB.btPersistentManifold_setCompanionIdB,
	PersistentManifold_setCompanionIdA = CLIB.btPersistentManifold_setCompanionIdA,
	DynamicsWorld_setConstraintSolver = CLIB.btDynamicsWorld_setConstraintSolver,
	GImpactMeshShapePart_TrimeshPrimitiveManager_getPart = CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_getPart,
	Generic6DofSpring2Constraint_getFrameOffsetA = CLIB.btGeneric6DofSpring2Constraint_getFrameOffsetA,
	BvhTree_new = CLIB.btBvhTree_new,
	ConvexPolyhedron_getFaces = CLIB.btConvexPolyhedron_getFaces,
	RigidBody_getNumConstraintRefs = CLIB.btRigidBody_getNumConstraintRefs,
	PersistentManifold_getContactPoint = CLIB.btPersistentManifold_getContactPoint,
	PersistentManifold_getBody1 = CLIB.btPersistentManifold_getBody1,
	GImpactShapeInterface_getPrimitiveManager = CLIB.btGImpactShapeInterface_getPrimitiveManager,
	SoftBody_RayFromToCaster_getRayFrom = CLIB.btSoftBody_RayFromToCaster_getRayFrom,
	QuantizedBvhTree_build_tree = CLIB.btQuantizedBvhTree_build_tree,
	OverlapCallback_delete = CLIB.btOverlapCallback_delete,
	PersistentManifold_clearUserCache = CLIB.btPersistentManifold_clearUserCache,
	CollisionWorld_RayResultCallback_delete = CLIB.btCollisionWorld_RayResultCallback_delete,
	PersistentManifold_clearManifold = CLIB.btPersistentManifold_clearManifold,
	RotationalLimitMotor2_getSpringDamping = CLIB.btRotationalLimitMotor2_getSpringDamping,
	PersistentManifold_new = CLIB.btPersistentManifold_new,
	BoxBoxDetector_getBox1 = CLIB.btBoxBoxDetector_getBox1,
	ContactSolverInfoData_setSor = CLIB.btContactSolverInfoData_setSor,
	SoftBody_appendLink = CLIB.btSoftBody_appendLink,
	HashedOverlappingPairCache_getOverlapFilterCallback = CLIB.btHashedOverlappingPairCache_getOverlapFilterCallback,
	Chunk_setChunkCode = CLIB.btChunk_setChunkCode,
	SliderConstraint_getRestitutionLimAng = CLIB.btSliderConstraint_getRestitutionLimAng,
	MultiBody_isAwake = CLIB.btMultiBody_isAwake,
	MultiBodySolverConstraint_setJacBindex = CLIB.btMultiBodySolverConstraint_setJacBindex,
	GImpactCollisionAlgorithm_CreateFunc_new = CLIB.btGImpactCollisionAlgorithm_CreateFunc_new,
	MultiBody_finalizeMultiDof = CLIB.btMultiBody_finalizeMultiDof,
	RotationalLimitMotor2_setServoMotor = CLIB.btRotationalLimitMotor2_setServoMotor,
	DbvtAabbMm_Classify = CLIB.btDbvtAabbMm_Classify,
	MultiBody_setupRevolute = CLIB.btMultiBody_setupRevolute,
	OverlappingPairCache_setOverlapFilterCallback = CLIB.btOverlappingPairCache_setOverlapFilterCallback,
	GImpactMeshShapePart_TrimeshPrimitiveManager_setMeshInterface = CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_setMeshInterface,
	OverlappingPairCache_processAllOverlappingPairs = CLIB.btOverlappingPairCache_processAllOverlappingPairs,
	ConeTwistConstraint_getFrameOffsetA = CLIB.btConeTwistConstraint_getFrameOffsetA,
	HingeConstraint_getAngularOnly = CLIB.btHingeConstraint_getAngularOnly,
	OverlappingPairCache_cleanOverlappingPair = CLIB.btOverlappingPairCache_cleanOverlappingPair,
	SliderConstraint_setSoftnessOrthoLin = CLIB.btSliderConstraint_setSoftnessOrthoLin,
	CollisionWorld_getCollisionObjectArray = CLIB.btCollisionWorld_getCollisionObjectArray,
	RigidBody_getMotionState = CLIB.btRigidBody_getMotionState,
	ConvexCast_delete = CLIB.btConvexCast_delete,
	OptimizedBvh_refit = CLIB.btOptimizedBvh_refit,
	SoftBody_updateConstants = CLIB.btSoftBody_updateConstants,
	NNCGConstraintSolver_getOnlyForNoneContact = CLIB.btNNCGConstraintSolver_getOnlyForNoneContact,
	SoftBody_Anchor_getBody = CLIB.btSoftBody_Anchor_getBody,
	MultiSphereShape_getSpherePosition = CLIB.btMultiSphereShape_getSpherePosition,
	CollisionAlgorithm_processCollision = CLIB.btCollisionAlgorithm_processCollision,
	SoftBody_getCfg = CLIB.btSoftBody_getCfg,
	RotationalLimitMotor_getNormalCFM = CLIB.btRotationalLimitMotor_getNormalCFM,
	MultimaterialTriangleMeshShape_getMaterialProperties = CLIB.btMultimaterialTriangleMeshShape_getMaterialProperties,
	MultimaterialTriangleMeshShape_new = CLIB.btMultimaterialTriangleMeshShape_new,
	MultiBodySliderConstraint_setPivotInB = CLIB.btMultiBodySliderConstraint_setPivotInB,
	MultiBodySliderConstraint_setPivotInA = CLIB.btMultiBodySliderConstraint_setPivotInA,
	MultiBodySliderConstraint_setJointAxis = CLIB.btMultiBodySliderConstraint_setJointAxis,
	RotationalLimitMotor_new2 = CLIB.btRotationalLimitMotor_new2,
	GImpactCollisionAlgorithm_getPart1 = CLIB.btGImpactCollisionAlgorithm_getPart1,
	MultiBodySliderConstraint_setFrameInB = CLIB.btMultiBodySliderConstraint_setFrameInB,
	MultibodyLink_getJointTorque = CLIB.btMultibodyLink_getJointTorque,
	MultiBodySliderConstraint_setFrameInA = CLIB.btMultiBodySliderConstraint_setFrameInA,
	Generic6DofSpring2Constraint_setMaxMotorForce = CLIB.btGeneric6DofSpring2Constraint_setMaxMotorForce,
	MultiBodySliderConstraint_getPivotInB = CLIB.btMultiBodySliderConstraint_getPivotInB,
	MultiBodySliderConstraint_getPivotInA = CLIB.btMultiBodySliderConstraint_getPivotInA,
	MultiBodySliderConstraint_getJointAxis = CLIB.btMultiBodySliderConstraint_getJointAxis,
	Box2dShape_getHalfExtentsWithoutMargin = CLIB.btBox2dShape_getHalfExtentsWithoutMargin,
	MultiBodySolverConstraint_delete = CLIB.btMultiBodySolverConstraint_delete,
	MultiBodySolverConstraint_setUpperLimit = CLIB.btMultiBodySolverConstraint_setUpperLimit,
	MultiBodySolverConstraint_setUnusedPadding4 = CLIB.btMultiBodySolverConstraint_setUnusedPadding4,
	MultiBodySolverConstraint_setSolverBodyIdB = CLIB.btMultiBodySolverConstraint_setSolverBodyIdB,
	Generic6DofSpring2Constraint_new2 = CLIB.btGeneric6DofSpring2Constraint_new2,
	MultiBodySolverConstraint_setSolverBodyIdA = CLIB.btMultiBodySolverConstraint_setSolverBodyIdA,
	MultiBodySolverConstraint_setRhsPenetration = CLIB.btMultiBodySolverConstraint_setRhsPenetration,
	MultiBodySolverConstraint_setRhs = CLIB.btMultiBodySolverConstraint_setRhs,
	GImpactMeshShapePart_new = CLIB.btGImpactMeshShapePart_new,
	ConvexShape_project = CLIB.btConvexShape_project,
	HingeConstraint_getLimitSoftness = CLIB.btHingeConstraint_getLimitSoftness,
	MultiBodySolverConstraint_setRelpos2CrossNormal = CLIB.btMultiBodySolverConstraint_setRelpos2CrossNormal,
	MultiBodySolverConstraint_setRelpos1CrossNormal = CLIB.btMultiBodySolverConstraint_setRelpos1CrossNormal,
	PolyhedralConvexShape_initializePolyhedralFeatures = CLIB.btPolyhedralConvexShape_initializePolyhedralFeatures,
	DbvtAabbMm_tMaxs = CLIB.btDbvtAabbMm_tMaxs,
	HingeConstraint_new3 = CLIB.btHingeConstraint_new3,
	AxisSweep3_getHandle = CLIB.btAxisSweep3_getHandle,
	AABB_new3 = CLIB.btAABB_new3,
	MultiBodySolverConstraint_setOriginalContactPoint = CLIB.btMultiBodySolverConstraint_setOriginalContactPoint,
	JointFeedback_setAppliedTorqueBodyB = CLIB.btJointFeedback_setAppliedTorqueBodyB,
	MultiBodySolverConstraint_setOrgConstraint = CLIB.btMultiBodySolverConstraint_setOrgConstraint,
	MultiBodySolverConstraint_setMultiBodyB = CLIB.btMultiBodySolverConstraint_setMultiBodyB,
	HingeConstraint_setFrames = CLIB.btHingeConstraint_setFrames,
	MultiBodySolverConstraint_setMultiBodyA = CLIB.btMultiBodySolverConstraint_setMultiBodyA,
	RigidBody_upcast = CLIB.btRigidBody_upcast,
	RotationalLimitMotor2_new2 = CLIB.btRotationalLimitMotor2_new2,
	MultiBodySolverConstraint_setLinkB = CLIB.btMultiBodySolverConstraint_setLinkB,
	TypedConstraint_getOverrideNumSolverIterations = CLIB.btTypedConstraint_getOverrideNumSolverIterations,
	MultiBodySolverConstraint_setJacDiagABInv = CLIB.btMultiBodySolverConstraint_setJacDiagABInv,
	OverlapFilterCallbackWrapper_new = CLIB.btOverlapFilterCallbackWrapper_new,
	SoftBodyWorldInfo_getSparsesdf = CLIB.btSoftBodyWorldInfo_getSparsesdf,
	CompoundCompoundCollisionAlgorithm_CreateFunc_new = CLIB.btCompoundCompoundCollisionAlgorithm_CreateFunc_new,
	QuantizedBvhTree_get_node_pointer = CLIB.btQuantizedBvhTree_get_node_pointer,
	MultiBodySolverConstraint_setFriction = CLIB.btMultiBodySolverConstraint_setFriction,
	MultiBodySolverConstraint_setDeltaVelBindex = CLIB.btMultiBodySolverConstraint_setDeltaVelBindex,
	MultiBodySolverConstraint_setContactNormal2 = CLIB.btMultiBodySolverConstraint_setContactNormal2,
	MultiBodySolverConstraint_setContactNormal1 = CLIB.btMultiBodySolverConstraint_setContactNormal1,
	UsageBitfield_setUsedVertexA = CLIB.btUsageBitfield_setUsedVertexA,
	ContactSolverInfoData_setSplitImpulseTurnErp = CLIB.btContactSolverInfoData_setSplitImpulseTurnErp,
	MultiBodySolverConstraint_setAppliedImpulse = CLIB.btMultiBodySolverConstraint_setAppliedImpulse,
	MultiBodySolverConstraint_setAngularComponentB = CLIB.btMultiBodySolverConstraint_setAngularComponentB,
	ManifoldResult_new = CLIB.btManifoldResult_new,
	MultiBodySolverConstraint_setAngularComponentA = CLIB.btMultiBodySolverConstraint_setAngularComponentA,
	CollisionWorld_LocalRayResult_getLocalShapeInfo = CLIB.btCollisionWorld_LocalRayResult_getLocalShapeInfo,
	AlignedObjectArray_btBroadphasePair_resizeNoInitialize = CLIB.btAlignedObjectArray_btBroadphasePair_resizeNoInitialize,
	MultiBodySolverConstraint_getUpperLimit = CLIB.btMultiBodySolverConstraint_getUpperLimit,
	GImpactCollisionAlgorithm_gimpact_vs_gimpact = CLIB.btGImpactCollisionAlgorithm_gimpact_vs_gimpact,
	MultiBodySolverConstraint_getUnusedPadding4 = CLIB.btMultiBodySolverConstraint_getUnusedPadding4,
	BroadphaseInterface_printStats = CLIB.btBroadphaseInterface_printStats,
	RotationalLimitMotor2_setLoLimit = CLIB.btRotationalLimitMotor2_setLoLimit,
	MultiBodySolverConstraint_getRhs = CLIB.btMultiBodySolverConstraint_getRhs,
	SliderConstraint_setRestitutionLimAng = CLIB.btSliderConstraint_setRestitutionLimAng,
	ConeTwistConstraint_setDamping = CLIB.btConeTwistConstraint_setDamping,
	BvhTriangleMeshShape_usesQuantizedAabbCompression = CLIB.btBvhTriangleMeshShape_usesQuantizedAabbCompression,
	CapsuleShape_getHalfHeight = CLIB.btCapsuleShape_getHalfHeight,
	GImpactBvh_boxQueryTrans = CLIB.btGImpactBvh_boxQueryTrans,
	PolyhedralConvexShape_getVertex = CLIB.btPolyhedralConvexShape_getVertex,
	SoftBody_sRayCast_new = CLIB.btSoftBody_sRayCast_new,
	MultiBodyFixedConstraint_setFrameInA = CLIB.btMultiBodyFixedConstraint_setFrameInA,
	CylinderShape_getHalfExtentsWithoutMargin = CLIB.btCylinderShape_getHalfExtentsWithoutMargin,
	SoftBody_SContact_new = CLIB.btSoftBody_SContact_new,
	MultiBodySolverConstraint_getLinkB = CLIB.btMultiBodySolverConstraint_getLinkB,
	GImpactMeshShapePart_getTrimeshPrimitiveManager = CLIB.btGImpactMeshShapePart_getTrimeshPrimitiveManager,
	CollisionWorld_ContactResultCallback_addSingleResult = CLIB.btCollisionWorld_ContactResultCallback_addSingleResult,
	MultiBodySolverConstraint_getLinkA = CLIB.btMultiBodySolverConstraint_getLinkA,
	MultiBodySolverConstraint_getJacBindex = CLIB.btMultiBodySolverConstraint_getJacBindex,
	MultiBodySolverConstraint_getFrictionIndex = CLIB.btMultiBodySolverConstraint_getFrictionIndex,
	QuantizedBvhNode_getEscapeIndexOrTriangleIndex = CLIB.btQuantizedBvhNode_getEscapeIndexOrTriangleIndex,
	MultiBodySolverConstraint_getContactNormal1 = CLIB.btMultiBodySolverConstraint_getContactNormal1,
	MultiBodySolverConstraint_getAngularComponentB = CLIB.btMultiBodySolverConstraint_getAngularComponentB,
	ConvexCast_CastResult_getHitTransformA = CLIB.btConvexCast_CastResult_getHitTransformA,
	HingeConstraint_getUseFrameOffset = CLIB.btHingeConstraint_getUseFrameOffset,
	ConvexTriangleCallback_setTriangleCount = CLIB.btConvexTriangleCallback_setTriangleCount,
	MultiBodyPoint2Point_setPivotInB = CLIB.btMultiBodyPoint2Point_setPivotInB,
	MultiBodyPoint2Point_getPivotInB = CLIB.btMultiBodyPoint2Point_getPivotInB,
	ContactSolverInfoData_setMaxErrorReduction = CLIB.btContactSolverInfoData_setMaxErrorReduction,
	CompoundShape_getUpdateRevision = CLIB.btCompoundShape_getUpdateRevision,
	CollisionWorld_ClosestConvexResultCallback_getHitPointWorld = CLIB.btCollisionWorld_ClosestConvexResultCallback_getHitPointWorld,
	Generic6DofConstraint_getAxis = CLIB.btGeneric6DofConstraint_getAxis,
	QuantizedBvh_new = CLIB.btQuantizedBvh_new,
	BoxBoxDetector_getBox2 = CLIB.btBoxBoxDetector_getBox2,
	MultiBodyLinkCollider_setLink = CLIB.btMultiBodyLinkCollider_setLink,
	MultiBodyLinkCollider_getMultiBody = CLIB.btMultiBodyLinkCollider_getMultiBody,
	BroadphaseInterface_rayTest3 = CLIB.btBroadphaseInterface_rayTest3,
	BroadphaseProxy_isConvex2d = CLIB.btBroadphaseProxy_isConvex2d,
	MultiBodyLinkCollider_new = CLIB.btMultiBodyLinkCollider_new,
	AxisSweep3_testAabbOverlap = CLIB.btAxisSweep3_testAabbOverlap,
	MultibodyLink_updateCacheMultiDof = CLIB.btMultibodyLink_updateCacheMultiDof,
	MultibodyLink_setUserPtr = CLIB.btMultibodyLink_setUserPtr,
	MultibodyLink_setZeroRotParentToThis = CLIB.btMultibodyLink_setZeroRotParentToThis,
	MultibodyLink_setParent = CLIB.btMultibodyLink_setParent,
	MultibodyLink_setMass = CLIB.btMultibodyLink_setMass,
	DbvtNode_setDataAsInt = CLIB.btDbvtNode_setDataAsInt,
	ConvexPlaneCollisionAlgorithm_CreateFunc_new = CLIB.btConvexPlaneCollisionAlgorithm_CreateFunc_new,
	SliderConstraint_getRestitutionLimLin = CLIB.btSliderConstraint_getRestitutionLimLin,
	MultibodyLink_setJointType = CLIB.btMultibodyLink_setJointType,
	MultibodyLink_setJointName = CLIB.btMultibodyLink_setJointName,
	ManifoldResult_getPersistentManifold = CLIB.btManifoldResult_getPersistentManifold,
	MultibodyLink_setJointFriction = CLIB.btMultibodyLink_setJointFriction,
	MultiBody_getAngularDamping = CLIB.btMultiBody_getAngularDamping,
	ManifoldPoint_setPartId1 = CLIB.btManifoldPoint_setPartId1,
	MultibodyLink_setFlags = CLIB.btMultibodyLink_setFlags,
	Dbvt_sStkNN_getB = CLIB.btDbvt_sStkNN_getB,
	SoftBody_setBUpdateRtCst = CLIB.btSoftBody_setBUpdateRtCst,
	MultibodyLink_setDVector = CLIB.btMultibodyLink_setDVector,
	MultibodyLink_setDofOffset = CLIB.btMultibodyLink_setDofOffset,
	MultibodyLink_setDofCount = CLIB.btMultibodyLink_setDofCount,
	ConeShape_getConeUpIndex = CLIB.btConeShape_getConeUpIndex,
	MultibodyLink_setCollider = CLIB.btMultibodyLink_setCollider,
	SoftBody_Impulse_getAsVelocity = CLIB.btSoftBody_Impulse_getAsVelocity,
	MultibodyLink_setCachedWorldTransform = CLIB.btMultibodyLink_setCachedWorldTransform,
	MultibodyLink_setCachedRVector = CLIB.btMultibodyLink_setCachedRVector,
	HingeConstraint_getEnableAngularMotor = CLIB.btHingeConstraint_getEnableAngularMotor,
	CollisionWorld_ContactResultCallback_needsCollision = CLIB.btCollisionWorld_ContactResultCallback_needsCollision,
	MultibodyLink_setCachedRotParentToThis = CLIB.btMultibodyLink_setCachedRotParentToThis,
	MultibodyLink_setAxisTop2 = CLIB.btMultibodyLink_setAxisTop2,
	MultibodyLink_setAxisBottom = CLIB.btMultibodyLink_setAxisBottom,
	MultiBody_setWorldToBaseRot = CLIB.btMultiBody_setWorldToBaseRot,
	ConstraintSolver_delete = CLIB.btConstraintSolver_delete,
	ShapeHull_getIndexPointer = CLIB.btShapeHull_getIndexPointer,
	MultibodyLink_setAppliedForce = CLIB.btMultibodyLink_setAppliedForce,
	SoftBody_updatePose = CLIB.btSoftBody_updatePose,
	MultibodyLink_setAppliedConstraintForce = CLIB.btMultibodyLink_setAppliedConstraintForce,
	MultibodyLink_getDVector = CLIB.btMultibodyLink_getDVector,
	BvhTriangleMeshShape_getOptimizedBvh = CLIB.btBvhTriangleMeshShape_getOptimizedBvh,
	CollisionWorld_ContactResultCallback_getCollisionFilterMask = CLIB.btCollisionWorld_ContactResultCallback_getCollisionFilterMask,
	ConvexShape_localGetSupportingVertexWithoutMargin = CLIB.btConvexShape_localGetSupportingVertexWithoutMargin,
	SoftBody_Cluster_getCom = CLIB.btSoftBody_Cluster_getCom,
	Generic6DofSpring2Constraint_enableMotor = CLIB.btGeneric6DofSpring2Constraint_enableMotor,
	CollisionObject_getSpinningFriction = CLIB.btCollisionObject_getSpinningFriction,
	RotationalLimitMotor2_getEnableSpring = CLIB.btRotationalLimitMotor2_getEnableSpring,
	SoftBody_Config_getKDP = CLIB.btSoftBody_Config_getKDP,
	ConeTwistConstraint_isMotorEnabled = CLIB.btConeTwistConstraint_isMotorEnabled,
	DefaultCollisionConstructionInfo_delete = CLIB.btDefaultCollisionConstructionInfo_delete,
	DefaultCollisionConstructionInfo_setCollisionAlgorithmPool = CLIB.btDefaultCollisionConstructionInfo_setCollisionAlgorithmPool,
	ConeTwistConstraint_calcAngleInfo = CLIB.btConeTwistConstraint_calcAngleInfo,
	DefaultCollisionConstructionInfo_getPersistentManifoldPool = CLIB.btDefaultCollisionConstructionInfo_getPersistentManifoldPool,
	SoftBody_Config_getCiterations = CLIB.btSoftBody_Config_getCiterations,
	DynamicsWorld_getWorldType = CLIB.btDynamicsWorld_getWorldType,
	AlignedObjectArray_btPersistentManifoldPtr_at = CLIB.btAlignedObjectArray_btPersistentManifoldPtr_at,
	AlignedObjectArray_btSoftBody_Link_resizeNoInitialize = CLIB.btAlignedObjectArray_btSoftBody_Link_resizeNoInitialize,
	RigidBody_btRigidBodyConstructionInfo_getMotionState = CLIB.btRigidBody_btRigidBodyConstructionInfo_getMotionState,
	ConvexPenetrationDepthSolver_calcPenDepth = CLIB.btConvexPenetrationDepthSolver_calcPenDepth,
	DbvtBroadphase_getCupdates = CLIB.btDbvtBroadphase_getCupdates,
	DbvtProxy_getStage = CLIB.btDbvtProxy_getStage,
	RotationalLimitMotor_setDamping = CLIB.btRotationalLimitMotor_setDamping,
	Dbvt_array_index_of = CLIB.btDbvt_array_index_of,
	Dbvt_update4 = CLIB.btDbvt_update4,
	Face_getIndices = CLIB.btFace_getIndices,
	MultiBody_setPosUpdated = CLIB.btMultiBody_setPosUpdated,
	Dbvt_remove = CLIB.btDbvt_remove,
	MultiBodySolverConstraint_getOverrideNumSolverIterations = CLIB.btMultiBodySolverConstraint_getOverrideNumSolverIterations,
	Hinge2Constraint_new = CLIB.btHinge2Constraint_new,
	RigidBody_getLocalInertia = CLIB.btRigidBody_getLocalInertia,
	GImpactQuantizedBvh_setPrimitiveManager = CLIB.btGImpactQuantizedBvh_setPrimitiveManager,
	ConvexCast_CastResult_setAllowedPenetration = CLIB.btConvexCast_CastResult_setAllowedPenetration,
	MaterialProperties_getMaterialBase = CLIB.btMaterialProperties_getMaterialBase,
	UniversalConstraint_getAxis1 = CLIB.btUniversalConstraint_getAxis1,
	AlignedObjectArray_btSoftBody_Node_size = CLIB.btAlignedObjectArray_btSoftBody_Node_size,
	MultiBody_addLinkConstraintTorque = CLIB.btMultiBody_addLinkConstraintTorque,
	CollisionShape_getMargin = CLIB.btCollisionShape_getMargin,
	MultiBody_addBaseForce = CLIB.btMultiBody_addBaseForce,
	BroadphasePair_getAlgorithm = CLIB.btBroadphasePair_getAlgorithm,
	Dbvt_allocate = CLIB.btDbvt_allocate,
	Dbvt_sStkNPS_getValue = CLIB.btDbvt_sStkNPS_getValue,
	SoftBody_setWorldInfo = CLIB.btSoftBody_setWorldInfo,
	CollisionShape_getContactBreakingThreshold = CLIB.btCollisionShape_getContactBreakingThreshold,
	AlignedObjectArray_btSoftBody_Face_at = CLIB.btAlignedObjectArray_btSoftBody_Face_at,
	Dbvt_sStkNP_setNode = CLIB.btDbvt_sStkNP_setNode,
	MultiBody_getNumPosVars = CLIB.btMultiBody_getNumPosVars,
	MultiBody_getBaseMass = CLIB.btMultiBody_getBaseMass,
	RotationalLimitMotor_getStopERP = CLIB.btRotationalLimitMotor_getStopERP,
	ConeShape_getRadius = CLIB.btConeShape_getRadius,
	DynamicsWorld_addRigidBody2 = CLIB.btDynamicsWorld_addRigidBody2,
	BroadphaseInterface_createProxy = CLIB.btBroadphaseInterface_createProxy,
	Generic6DofConstraint_getRotationalLimitMotor = CLIB.btGeneric6DofConstraint_getRotationalLimitMotor,
	Generic6DofConstraint_getAngularLowerLimit = CLIB.btGeneric6DofConstraint_getAngularLowerLimit,
	DefaultCollisionConstructionInfo_getCustomCollisionAlgorithmMaxElementSize = CLIB.btDefaultCollisionConstructionInfo_getCustomCollisionAlgorithmMaxElementSize,
	TypedConstraint_btConstraintInfo2_getCfm = CLIB.btTypedConstraint_btConstraintInfo2_getCfm,
	MultiBodyDynamicsWorld_getMultiBodyConstraint = CLIB.btMultiBodyDynamicsWorld_getMultiBodyConstraint,
	Dbvt_sStkCLN_new = CLIB.btDbvt_sStkCLN_new,
	ManifoldResult_calculateCombinedContactDamping = CLIB.btManifoldResult_calculateCombinedContactDamping,
	RotationalLimitMotor_getBounce = CLIB.btRotationalLimitMotor_getBounce,
	CollisionShape_isPolyhedral = CLIB.btCollisionShape_isPolyhedral,
	HingeConstraint_new2 = CLIB.btHingeConstraint_new2,
	CompoundCompoundCollisionAlgorithm_new = CLIB.btCompoundCompoundCollisionAlgorithm_new,
	GImpactCollisionAlgorithm_gimpact_vs_compoundshape = CLIB.btGImpactCollisionAlgorithm_gimpact_vs_compoundshape,
	CollisionObject_getContactDamping = CLIB.btCollisionObject_getContactDamping,
	ManifoldPoint_setCombinedRollingFriction = CLIB.btManifoldPoint_setCombinedRollingFriction,
	DbvtAabbMm_SignedExpand = CLIB.btDbvtAabbMm_SignedExpand,
	DbvtAabbMm_FromPoints = CLIB.btDbvtAabbMm_FromPoints,
	DefaultSoftBodySolver_new = CLIB.btDefaultSoftBodySolver_new,
	CompoundCollisionAlgorithm_getChildAlgorithm = CLIB.btCompoundCollisionAlgorithm_getChildAlgorithm,
	RotationalLimitMotor2_getEquilibriumPoint = CLIB.btRotationalLimitMotor2_getEquilibriumPoint,
	ManifoldPoint_getLifeTime = CLIB.btManifoldPoint_getLifeTime,
	CollisionWorld_AllHitsRayResultCallback_getHitPointWorld = CLIB.btCollisionWorld_AllHitsRayResultCallback_getHitPointWorld,
	Generic6DofSpring2Constraint_setEquilibriumPoint2 = CLIB.btGeneric6DofSpring2Constraint_setEquilibriumPoint2,
	ManifoldPoint_getContactERP = CLIB.btManifoldPoint_getContactERP,
	Dbvt_setLeaves = CLIB.btDbvt_setLeaves,
	DbvtBroadphase_setFixedleft = CLIB.btDbvtBroadphase_setFixedleft,
	DynamicsWorld_getGravity = CLIB.btDynamicsWorld_getGravity,
	Convex2dConvex2dAlgorithm_CreateFunc_new = CLIB.btConvex2dConvex2dAlgorithm_CreateFunc_new,
	ContactSolverInfoData_setErp = CLIB.btContactSolverInfoData_setErp,
	GImpactBvh_getNodeTriangle = CLIB.btGImpactBvh_getNodeTriangle,
	ConvexShape_getAabbNonVirtual = CLIB.btConvexShape_getAabbNonVirtual,
	BroadphaseProxy_isSoftBody = CLIB.btBroadphaseProxy_isSoftBody,
	DbvtBroadphase_getPrediction = CLIB.btDbvtBroadphase_getPrediction,
	AngularLimit_getHalfRange = CLIB.btAngularLimit_getHalfRange,
	RotationalLimitMotor_getCurrentPosition = CLIB.btRotationalLimitMotor_getCurrentPosition,
	CollisionWorld_ClosestConvexResultCallback_setConvexToWorld = CLIB.btCollisionWorld_ClosestConvexResultCallback_setConvexToWorld,
	ConvexConvexAlgorithm_CreateFunc_setPdSolver = CLIB.btConvexConvexAlgorithm_CreateFunc_setPdSolver,
	GImpactMeshShapePart_getPart = CLIB.btGImpactMeshShapePart_getPart,
	SliderConstraint_setFrames = CLIB.btSliderConstraint_setFrames,
	ContactSolverInfoData_getRestitution = CLIB.btContactSolverInfoData_getRestitution,
	RigidBody_proceedToTransform = CLIB.btRigidBody_proceedToTransform,
	SoftBody_initializeFaceTree = CLIB.btSoftBody_initializeFaceTree,
	MultiBody_isUsingGlobalVelocities = CLIB.btMultiBody_isUsingGlobalVelocities,
	ContactSolverInfoData_getSplitImpulse = CLIB.btContactSolverInfoData_getSplitImpulse,
	CollisionDispatcher_setDispatcherFlags = CLIB.btCollisionDispatcher_setDispatcherFlags,
	CollisionWorld_ConvexResultCallback_getCollisionFilterMask = CLIB.btCollisionWorld_ConvexResultCallback_getCollisionFilterMask,
	CollisionObject_setIslandTag = CLIB.btCollisionObject_setIslandTag,
	ConeShapeZ_new = CLIB.btConeShapeZ_new,
	ConeTwistConstraint_setMaxMotorImpulseNormalized = CLIB.btConeTwistConstraint_setMaxMotorImpulseNormalized,
	MultibodyLink_getLinkName = CLIB.btMultibodyLink_getLinkName,
	CollisionWorld_ClosestRayResultCallback_getRayFromWorld = CLIB.btCollisionWorld_ClosestRayResultCallback_getRayFromWorld,
	Dbvt_sStkNN_new = CLIB.btDbvt_sStkNN_new,
	ConvexCast_CastResult_setDebugDrawer = CLIB.btConvexCast_CastResult_setDebugDrawer,
	AlignedObjectArray_btSoftBody_Anchor_push_back = CLIB.btAlignedObjectArray_btSoftBody_Anchor_push_back,
	ConvexHullShape_getNumPoints = CLIB.btConvexHullShape_getNumPoints,
	Convex2dConvex2dAlgorithm_CreateFunc_setNumPerturbationIterations = CLIB.btConvex2dConvex2dAlgorithm_CreateFunc_setNumPerturbationIterations,
	RotationalLimitMotor_getCurrentLimitError = CLIB.btRotationalLimitMotor_getCurrentLimitError,
	DynamicsWorld_stepSimulation = CLIB.btDynamicsWorld_stepSimulation,
	SortedOverlappingPairCache_getOverlapFilterCallback = CLIB.btSortedOverlappingPairCache_getOverlapFilterCallback,
	GImpactMeshShapePart_TrimeshPrimitiveManager_new3 = CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_new3,
	HingeConstraint_setAngularOnly = CLIB.btHingeConstraint_setAngularOnly,
	CylinderShapeX_new2 = CLIB.btCylinderShapeX_new2,
	AlignedObjectArray_btBroadphasePair_at = CLIB.btAlignedObjectArray_btBroadphasePair_at,
	ManifoldResult_setClosestPointDistanceThreshold = CLIB.btManifoldResult_setClosestPointDistanceThreshold,
	RigidBody_getLinearVelocity = CLIB.btRigidBody_getLinearVelocity,
	GImpactQuantizedBvh_new = CLIB.btGImpactQuantizedBvh_new,
	TypedConstraint_btConstraintInfo2_getJ1linearAxis = CLIB.btTypedConstraint_btConstraintInfo2_getJ1linearAxis,
	HingeConstraint_getHingeAngle2 = CLIB.btHingeConstraint_getHingeAngle2,
	CollisionWorld_AllHitsRayResultCallback_getRayFromWorld = CLIB.btCollisionWorld_AllHitsRayResultCallback_getRayFromWorld,
	HingeConstraint_getBFrame = CLIB.btHingeConstraint_getBFrame,
	ConvexCast_CastResult_getHitTransformB = CLIB.btConvexCast_CastResult_getHitTransformB,
	Convex2dShape_new = CLIB.btConvex2dShape_new,
	Convex2dConvex2dAlgorithm_new = CLIB.btConvex2dConvex2dAlgorithm_new,
	IndexedMesh_setIndexType = CLIB.btIndexedMesh_setIndexType,
	DbvtBroadphase_getUpdates_call = CLIB.btDbvtBroadphase_getUpdates_call,
	ManifoldResult_getBody0Internal = CLIB.btManifoldResult_getBody0Internal,
	SoftBody_Cluster_getLeaf = CLIB.btSoftBody_Cluster_getLeaf,
	Dbvt_sStkNN_new2 = CLIB.btDbvt_sStkNN_new2,
	ContactSolverInfoData_setWarmstartingFactor = CLIB.btContactSolverInfoData_setWarmstartingFactor,
	MultiBodyDynamicsWorld_integrateTransforms = CLIB.btMultiBodyDynamicsWorld_integrateTransforms,
	MultibodyLink_getCachedRVector = CLIB.btMultibodyLink_getCachedRVector,
	ConvexInternalShape_getMarginNV = CLIB.btConvexInternalShape_getMarginNV,
	CollisionObject_setHitFraction = CLIB.btCollisionObject_setHitFraction,
	GImpactShapeInterface_getGImpactShapeType = CLIB.btGImpactShapeInterface_getGImpactShapeType,
	ConvexPolyhedron_getLocalCenter = CLIB.btConvexPolyhedron_getLocalCenter,
	ConvexConvexAlgorithm_CreateFunc_getNumPerturbationIterations = CLIB.btConvexConvexAlgorithm_CreateFunc_getNumPerturbationIterations,
	DbvtProxy_getLeaf = CLIB.btDbvtProxy_getLeaf,
	MultiBody_calcAccelerationDeltasMultiDof = CLIB.btMultiBody_calcAccelerationDeltasMultiDof,
	ContactSolverInfoData_setRestingContactRestitutionThreshold = CLIB.btContactSolverInfoData_setRestingContactRestitutionThreshold,
	GjkPairDetector_getFixContactNormalDirection = CLIB.btGjkPairDetector_getFixContactNormalDirection,
	ContactSolverInfoData_setLinearSlop = CLIB.btContactSolverInfoData_setLinearSlop,
	GImpactMeshShapePart_TrimeshPrimitiveManager_getScale = CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_getScale,
	SoftBody_getFaces = CLIB.btSoftBody_getFaces,
	VoronoiSimplexSolver_getCachedP1 = CLIB.btVoronoiSimplexSolver_getCachedP1,
	RigidBody_getLinearSleepingThreshold = CLIB.btRigidBody_getLinearSleepingThreshold,
	Convex2dConvex2dAlgorithm_CreateFunc_getSimplexSolver = CLIB.btConvex2dConvex2dAlgorithm_CreateFunc_getSimplexSolver,
	ContactSolverInfoData_getSplitImpulsePenetrationThreshold = CLIB.btContactSolverInfoData_getSplitImpulsePenetrationThreshold,
	GImpactQuantizedBvh_getNodeCount = CLIB.btGImpactQuantizedBvh_getNodeCount,
	ContactSolverInfoData_getSolverMode = CLIB.btContactSolverInfoData_getSolverMode,
	BoxBoxCollisionAlgorithm_CreateFunc_new = CLIB.btBoxBoxCollisionAlgorithm_CreateFunc_new,
	ManifoldPoint_setPartId0 = CLIB.btManifoldPoint_setPartId0,
	GImpactMeshShapePart_TrimeshPrimitiveManager_lock = CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_lock,
	ConvexPolyhedron_setME = CLIB.btConvexPolyhedron_setME,
	MultiBodySolverConstraint_new = CLIB.btMultiBodySolverConstraint_new,
	ContactSolverInfoData_getMaxGyroscopicForce = CLIB.btContactSolverInfoData_getMaxGyroscopicForce,
	DiscreteCollisionDetectorInterface_Result_addContactPoint = CLIB.btDiscreteCollisionDetectorInterface_Result_addContactPoint,
	ContactSolverInfoData_getDamping = CLIB.btContactSolverInfoData_getDamping,
	DefaultCollisionConstructionInfo_getDefaultMaxCollisionAlgorithmPoolSize = CLIB.btDefaultCollisionConstructionInfo_getDefaultMaxCollisionAlgorithmPoolSize,
	RotationalLimitMotor_getDamping = CLIB.btRotationalLimitMotor_getDamping,
	ConeTwistConstraint_setLimit = CLIB.btConeTwistConstraint_setLimit,
	DbvtBroadphase_getCid = CLIB.btDbvtBroadphase_getCid,
	ConeTwistConstraint_getTwistSpan = CLIB.btConeTwistConstraint_getTwistSpan,
	DbvtAabbMm_FromCR = CLIB.btDbvtAabbMm_FromCR,
	GhostObject_upcast = CLIB.btGhostObject_upcast,
	BroadphaseProxy_setUniqueId = CLIB.btBroadphaseProxy_setUniqueId,
	CollisionAlgorithmConstructionInfo_setManifold = CLIB.btCollisionAlgorithmConstructionInfo_setManifold,
	FixedConstraint_new = CLIB.btFixedConstraint_new,
	TriangleMeshShape_getLocalAabbMin = CLIB.btTriangleMeshShape_getLocalAabbMin,
	CollisionWorld_LocalConvexResult_setHitPointLocal = CLIB.btCollisionWorld_LocalConvexResult_setHitPointLocal,
	MultiBodySolverConstraint_getOrgConstraint = CLIB.btMultiBodySolverConstraint_getOrgConstraint,
	BroadphaseProxy_setCollisionFilterGroup = CLIB.btBroadphaseProxy_setCollisionFilterGroup,
	GImpactShapeInterface_postUpdate = CLIB.btGImpactShapeInterface_postUpdate,
	CollisionObjectWrapper_setCollisionObject = CLIB.btCollisionObjectWrapper_setCollisionObject,
	UsageBitfield_setUsedVertexB = CLIB.btUsageBitfield_setUsedVertexB,
	GhostObject_getOverlappingPairs = CLIB.btGhostObject_getOverlappingPairs,
	ConeTwistConstraint_getInfo1NonVirtual = CLIB.btConeTwistConstraint_getInfo1NonVirtual,
	DefaultCollisionConstructionInfo_setDefaultMaxCollisionAlgorithmPoolSize = CLIB.btDefaultCollisionConstructionInfo_setDefaultMaxCollisionAlgorithmPoolSize,
	GImpactShapeInterface_getNumChildShapes = CLIB.btGImpactShapeInterface_getNumChildShapes,
	GImpactShapeInterface_getChildTransform = CLIB.btGImpactShapeInterface_getChildTransform,
	DefaultMotionState_getGraphicsWorldTrans = CLIB.btDefaultMotionState_getGraphicsWorldTrans,
	Dbvt_sStkNPS_setValue = CLIB.btDbvt_sStkNPS_setValue,
	DiscreteDynamicsWorld_new = CLIB.btDiscreteDynamicsWorld_new,
	CollisionWorld_ClosestConvexResultCallback_getHitCollisionObject = CLIB.btCollisionWorld_ClosestConvexResultCallback_getHitCollisionObject,
	ConeTwistConstraint_getSwingSpan2 = CLIB.btConeTwistConstraint_getSwingSpan2,
	GImpactQuantizedBvh_rayQuery = CLIB.btGImpactQuantizedBvh_rayQuery,
	BroadphaseRayCallback_getRayDirectionInverse = CLIB.btBroadphaseRayCallback_getRayDirectionInverse,
	ManifoldPoint_getAppliedImpulseLateral2 = CLIB.btManifoldPoint_getAppliedImpulseLateral2,
	ConvexInternalShape_setSafeMargin = CLIB.btConvexInternalShape_setSafeMargin,
	DbvtBroadphase_setFupdates = CLIB.btDbvtBroadphase_setFupdates,
	ConvexConvexAlgorithm_new = CLIB.btConvexConvexAlgorithm_new,
	Generic6DofConstraint_getFrameOffsetA = CLIB.btGeneric6DofConstraint_getFrameOffsetA,
	CompoundShape_removeChildShape = CLIB.btCompoundShape_removeChildShape,
	CompoundShapeChild_getChildShape = CLIB.btCompoundShapeChild_getChildShape,
	CylinderShape_getHalfExtentsWithMargin = CLIB.btCylinderShape_getHalfExtentsWithMargin,
	CollisionObject_new = CLIB.btCollisionObject_new,
	CompoundShape_recalculateLocalAabb = CLIB.btCompoundShape_recalculateLocalAabb,
	TranslationalLimitMotor_getRestitution = CLIB.btTranslationalLimitMotor_getRestitution,
	CollisionAlgorithm_calculateTimeOfImpact = CLIB.btCollisionAlgorithm_calculateTimeOfImpact,
	DispatcherInfo_getUseContinuous = CLIB.btDispatcherInfo_getUseContinuous,
	MultiBody_internalNeedsJointFeedback = CLIB.btMultiBody_internalNeedsJointFeedback,
	Chunk_getLength = CLIB.btChunk_getLength,
	CompoundShapeChild_setChildShape = CLIB.btCompoundShapeChild_setChildShape,
	CollisionObject_setBroadphaseHandle = CLIB.btCollisionObject_setBroadphaseHandle,
	CompoundShapeChild_setChildMargin = CLIB.btCompoundShapeChild_setChildMargin,
	MultibodyLink_getJointFriction = CLIB.btMultibodyLink_getJointFriction,
	CollisionShape_setMargin = CLIB.btCollisionShape_setMargin,
	TranslationalLimitMotor_testLimitValue = CLIB.btTranslationalLimitMotor_testLimitValue,
	Dispatcher_delete = CLIB.btDispatcher_delete,
	CollisionObject_delete = CLIB.btCollisionObject_delete,
	Dbvt_ICollide_new = CLIB.btDbvt_ICollide_new,
	ConvexPlaneCollisionAlgorithm_CreateFunc_getNumPerturbationIterations = CLIB.btConvexPlaneCollisionAlgorithm_CreateFunc_getNumPerturbationIterations,
	Generic6DofConstraint_getLinearLowerLimit = CLIB.btGeneric6DofConstraint_getLinearLowerLimit,
	DynamicsWorld_clearForces = CLIB.btDynamicsWorld_clearForces,
	CollisionWorld_LocalConvexResult_getHitFraction = CLIB.btCollisionWorld_LocalConvexResult_getHitFraction,
	Dispatcher_getInternalManifoldPool = CLIB.btDispatcher_getInternalManifoldPool,
	Point2PointConstraint_new2 = CLIB.btPoint2PointConstraint_new2,
	SoftBody_Link_getBbending = CLIB.btSoftBody_Link_getBbending,
	CollisionWorld_convexSweepTest = CLIB.btCollisionWorld_convexSweepTest,
	QuantizedBvh_quantizeWithClamp = CLIB.btQuantizedBvh_quantizeWithClamp,
	MultiBody_setUserPointer = CLIB.btMultiBody_setUserPointer,
	GImpactMeshShapePart_TrimeshPrimitiveManager_setNumverts = CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_setNumverts,
	BroadphaseInterface_resetPool = CLIB.btBroadphaseInterface_resetPool,
	SimulationIslandManager_initUnionFind = CLIB.btSimulationIslandManager_initUnionFind,
	Generic6DofSpringConstraint_setEquilibriumPoint = CLIB.btGeneric6DofSpringConstraint_setEquilibriumPoint,
	DefaultMotionState_getUserPointer = CLIB.btDefaultMotionState_getUserPointer,
	AABB_new4 = CLIB.btAABB_new4,
	DiscreteDynamicsWorld_getSimulationIslandManager = CLIB.btDiscreteDynamicsWorld_getSimulationIslandManager,
	ManifoldPoint_setContactCFM = CLIB.btManifoldPoint_setContactCFM,
	DispatcherInfo_setDebugDraw = CLIB.btDispatcherInfo_setDebugDraw,
	CollisionObject_getCcdSquareMotionThreshold = CLIB.btCollisionObject_getCcdSquareMotionThreshold,
	GImpactBvh_getPrimitiveManager = CLIB.btGImpactBvh_getPrimitiveManager,
	ConvexShape_getMarginNonVirtual = CLIB.btConvexShape_getMarginNonVirtual,
	Dbvt_getRoot = CLIB.btDbvt_getRoot,
	BroadphaseAabbCallbackWrapper_new = CLIB.btBroadphaseAabbCallbackWrapper_new,
	CapsuleShape_getRadius = CLIB.btCapsuleShape_getRadius,
	CollisionObject_setRollingFriction = CLIB.btCollisionObject_setRollingFriction,
	GImpactQuantizedBvh_setNodeBound = CLIB.btGImpactQuantizedBvh_setNodeBound,
	BroadphaseProxy_delete = CLIB.btBroadphaseProxy_delete,
	AABB_new = CLIB.btAABB_new,
	Generic6DofSpring2Constraint_matrixToEulerZXY = CLIB.btGeneric6DofSpring2Constraint_matrixToEulerZXY,
	MultibodyLink_getZeroRotParentToThis = CLIB.btMultibodyLink_getZeroRotParentToThis,
	TriangleMeshShape_recalcLocalAabb = CLIB.btTriangleMeshShape_recalcLocalAabb,
	Generic6DofSpring2Constraint_setStiffness = CLIB.btGeneric6DofSpring2Constraint_setStiffness,
	CollisionWorld_RayResultCallback_setClosestHitFraction = CLIB.btCollisionWorld_RayResultCallback_setClosestHitFraction,
	TypedConstraint_btConstraintInfo2_getDamping = CLIB.btTypedConstraint_btConstraintInfo2_getDamping,
	ConeTwistConstraint_getMotorTarget = CLIB.btConeTwistConstraint_getMotorTarget,
	TriangleIndexVertexArray_new2 = CLIB.btTriangleIndexVertexArray_new2,
	TypedConstraint_setUserConstraintId = CLIB.btTypedConstraint_setUserConstraintId,
	BroadphaseProxy_isConcave = CLIB.btBroadphaseProxy_isConcave,
	Dbvt_ICollide_Process2 = CLIB.btDbvt_ICollide_Process2,
	MultiBodyFixedConstraint_getPivotInA = CLIB.btMultiBodyFixedConstraint_getPivotInA,
	QuantizedBvhTree_getLeftNode = CLIB.btQuantizedBvhTree_getLeftNode,
	SoftBody_Pose_getVolume = CLIB.btSoftBody_Pose_getVolume,
	ConstraintSetting_getTau = CLIB.btConstraintSetting_getTau,
	GImpactMeshShapePart_TrimeshPrimitiveManager_setVertexbase = CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_setVertexbase,
	BroadphasePair_getPProxy1 = CLIB.btBroadphasePair_getPProxy1,
	SoftBody_SContact_getWeights = CLIB.btSoftBody_SContact_getWeights,
	SoftBodyHelpers_CreateFromConvexHull = CLIB.btSoftBodyHelpers_CreateFromConvexHull,
	BroadphaseProxy_getAabbMax = CLIB.btBroadphaseProxy_getAabbMax,
	AlignedObjectArray_btSoftBody_Link_at = CLIB.btAlignedObjectArray_btSoftBody_Link_at,
	CollisionShape_setLocalScaling = CLIB.btCollisionShape_setLocalScaling,
	SoftBody_getLinkVertexNormalData = CLIB.btSoftBody_getLinkVertexNormalData,
	SoftBody_addAeroForceToNode = CLIB.btSoftBody_addAeroForceToNode,
	BroadphaseInterface_getAabb = CLIB.btBroadphaseInterface_getAabb,
	CollisionWorld_ClosestConvexResultCallback_setConvexFromWorld = CLIB.btCollisionWorld_ClosestConvexResultCallback_setConvexFromWorld,
	GImpactShapeInterface_getPrimitiveTriangle = CLIB.btGImpactShapeInterface_getPrimitiveTriangle,
	MultiBodyPoint2Point_new2 = CLIB.btMultiBodyPoint2Point_new2,
	Dispatcher_clearManifold = CLIB.btDispatcher_clearManifold,
	GImpactCollisionAlgorithm_new = CLIB.btGImpactCollisionAlgorithm_new,
	BroadphasePair_getPProxy0 = CLIB.btBroadphasePair_getPProxy0,
	ManifoldResult_refreshContactPoints = CLIB.btManifoldResult_refreshContactPoints,
	RotationalLimitMotor_setNormalCFM = CLIB.btRotationalLimitMotor_setNormalCFM,
	TranslationalLimitMotor2_getCurrentLimitError = CLIB.btTranslationalLimitMotor2_getCurrentLimitError,
	SoftBodySolver_getTimeScale = CLIB.btSoftBodySolver_getTimeScale,
	Dbvt_IWriter_delete = CLIB.btDbvt_IWriter_delete,
	Dbvt_sStkNPS_new2 = CLIB.btDbvt_sStkNPS_new2,
	ConvexPointCloudShape_setPoints = CLIB.btConvexPointCloudShape_setPoints,
	CollisionWorld_contactTest = CLIB.btCollisionWorld_contactTest,
	ConvexConvexAlgorithm_CreateFunc_getMinimumPointsPerturbationThreshold = CLIB.btConvexConvexAlgorithm_CreateFunc_getMinimumPointsPerturbationThreshold,
	MultiBody_getNumDofs = CLIB.btMultiBody_getNumDofs,
	ContactSolverInfoData_setErp2 = CLIB.btContactSolverInfoData_setErp2,
	TranslationalLimitMotor_setRestitution = CLIB.btTranslationalLimitMotor_setRestitution,
	CollisionShape_setUserPointer = CLIB.btCollisionShape_setUserPointer,
	SoftBodySolver_solveConstraints = CLIB.btSoftBodySolver_solveConstraints,
	CollisionWorld_objectQuerySingle = CLIB.btCollisionWorld_objectQuerySingle,
	DbvtAabbMm_tMins = CLIB.btDbvtAabbMm_tMins,
	DiscreteCollisionDetectorInterface_ClosestPointInput_getMaximumDistanceSquared = CLIB.btDiscreteCollisionDetectorInterface_ClosestPointInput_getMaximumDistanceSquared,
	RotationalLimitMotor2_getSpringStiffness = CLIB.btRotationalLimitMotor2_getSpringStiffness,
	ManifoldPoint_getIndex1 = CLIB.btManifoldPoint_getIndex1,
	OptimizedBvh_refitPartial = CLIB.btOptimizedBvh_refitPartial,
	SoftBody_addForce2 = CLIB.btSoftBody_addForce2,
	Box2dBox2dCollisionAlgorithm_new = CLIB.btBox2dBox2dCollisionAlgorithm_new,
	RotationalLimitMotor2_getMotorERP = CLIB.btRotationalLimitMotor2_getMotorERP,
	ConeShape_setHeight = CLIB.btConeShape_setHeight,
	CollisionObject_getWorldArrayIndex = CLIB.btCollisionObject_getWorldArrayIndex,
	SoftBody_defaultCollisionHandler = CLIB.btSoftBody_defaultCollisionHandler,
	ConeTwistConstraint_getDamping = CLIB.btConeTwistConstraint_getDamping,
	BoxBoxDetector_setBox1 = CLIB.btBoxBoxDetector_setBox1,
	CollisionWorld_LocalRayResult_setHitFraction = CLIB.btCollisionWorld_LocalRayResult_setHitFraction,
	QuantizedBvhTree_clearNodes = CLIB.btQuantizedBvhTree_clearNodes,
	CollisionObject_getDeactivationTime = CLIB.btCollisionObject_getDeactivationTime,
	AlignedObjectArray_btSoftBody_Tetra_push_back = CLIB.btAlignedObjectArray_btSoftBody_Tetra_push_back,
	RotationalLimitMotor2_setSpringDampingLimited = CLIB.btRotationalLimitMotor2_setSpringDampingLimited,
	TranslationalLimitMotor_setAccumulatedImpulse = CLIB.btTranslationalLimitMotor_setAccumulatedImpulse,
	DefaultCollisionConfiguration_setPlaneConvexMultipointIterations = CLIB.btDefaultCollisionConfiguration_setPlaneConvexMultipointIterations,
	MultiBodyConstraint_getIslandIdA = CLIB.btMultiBodyConstraint_getIslandIdA,
	PointCollector_setPointInWorld = CLIB.btPointCollector_setPointInWorld,
	ContactSolverInfoData_setTau = CLIB.btContactSolverInfoData_setTau,
	QuantizedBvhTree_getNodeBound = CLIB.btQuantizedBvhTree_getNodeBound,
	AlignedObjectArray_btSoftBody_Link_size = CLIB.btAlignedObjectArray_btSoftBody_Link_size,
	CompoundShape_addChildShape = CLIB.btCompoundShape_addChildShape,
	SoftBody_evaluateCom = CLIB.btSoftBody_evaluateCom,
	ConvexPlaneCollisionAlgorithm_CreateFunc_getMinimumPointsPerturbationThreshold = CLIB.btConvexPlaneCollisionAlgorithm_CreateFunc_getMinimumPointsPerturbationThreshold,
	BoxShape_getPlaneEquation = CLIB.btBoxShape_getPlaneEquation,
	GImpactBvh_update = CLIB.btGImpactBvh_update,
	TranslationalLimitMotor2_getBounce = CLIB.btTranslationalLimitMotor2_getBounce,
	AlignedObjectArray_btSoftBody_Node_resizeNoInitialize = CLIB.btAlignedObjectArray_btSoftBody_Node_resizeNoInitialize,
	Convex2dConvex2dAlgorithm_CreateFunc_getNumPerturbationIterations = CLIB.btConvex2dConvex2dAlgorithm_CreateFunc_getNumPerturbationIterations,
	ConvexShape_getAabbSlow = CLIB.btConvexShape_getAabbSlow,
	MultibodyLink_getAbsFrameTotVelocity = CLIB.btMultibodyLink_getAbsFrameTotVelocity,
	GImpactMeshShapePart_TrimeshPrimitiveManager_getLock_count = CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_getLock_count,
	MultiBody_setMaxAppliedImpulse = CLIB.btMultiBody_setMaxAppliedImpulse,
	ContactSolverInfoData_getFrictionErp = CLIB.btContactSolverInfoData_getFrictionErp,
	BoxBoxCollisionAlgorithm_new = CLIB.btBoxBoxCollisionAlgorithm_new,
	MultiBodyConstraint_delete = CLIB.btMultiBodyConstraint_delete,
	CollisionObjectWrapper_getIndex = CLIB.btCollisionObjectWrapper_getIndex,
	SliderConstraint_setRestitutionLimLin = CLIB.btSliderConstraint_setRestitutionLimLin,
	CollisionWorld_ClosestRayResultCallback_getHitPointWorld = CLIB.btCollisionWorld_ClosestRayResultCallback_getHitPointWorld,
	PairSet_push_pair_inv = CLIB.btPairSet_push_pair_inv,
	Hinge2Constraint_setLowerLimit = CLIB.btHinge2Constraint_setLowerLimit,
	AlignedObjectArray_btBroadphasePair_size = CLIB.btAlignedObjectArray_btBroadphasePair_size,
	BroadphaseRayCallback_setLambda_max = CLIB.btBroadphaseRayCallback_setLambda_max,
	CollisionWorld_AllHitsRayResultCallback_setRayToWorld = CLIB.btCollisionWorld_AllHitsRayResultCallback_setRayToWorld,
	DbvtBroadphase_setNeedcleanup = CLIB.btDbvtBroadphase_setNeedcleanup,
	Dbvt_optimizeTopDown = CLIB.btDbvt_optimizeTopDown,
	DbvtBroadphase_optimize = CLIB.btDbvtBroadphase_optimize,
	CollisionObject_getInterpolationAngularVelocity = CLIB.btCollisionObject_getInterpolationAngularVelocity,
	RigidBody_applyGravity = CLIB.btRigidBody_applyGravity,
	ManifoldPoint_getFrictionCFM = CLIB.btManifoldPoint_getFrictionCFM,
	TriangleCallbackWrapper_new = CLIB.btTriangleCallbackWrapper_new,
	ConvexInternalShape_setSafeMargin2 = CLIB.btConvexInternalShape_setSafeMargin2,
	CollisionObjectWrapper_setParent = CLIB.btCollisionObjectWrapper_setParent,
	CollisionObject_activate = CLIB.btCollisionObject_activate,
	Generic6DofSpring2Constraint_setLimit = CLIB.btGeneric6DofSpring2Constraint_setLimit,
	ContactSolverInfoData_setFrictionCfm = CLIB.btContactSolverInfoData_setFrictionCfm,
	MultibodyLink_getCollider = CLIB.btMultibodyLink_getCollider,
	GImpactQuantizedBvh_getNodeBound = CLIB.btGImpactQuantizedBvh_getNodeBound,
	ManifoldPoint_getDistance = CLIB.btManifoldPoint_getDistance,
	CollisionWorld_ConvexResultCallback_hasHit = CLIB.btCollisionWorld_ConvexResultCallback_hasHit,
	ConeTwistConstraint_getLimit = CLIB.btConeTwistConstraint_getLimit,
	MultiBody_serialize = CLIB.btMultiBody_serialize,
	MultiBodyDynamicsWorld_new = CLIB.btMultiBodyDynamicsWorld_new,
	BroadphaseInterface_setAabb = CLIB.btBroadphaseInterface_setAabb,
	SparseSdf3_Initialize = CLIB.btSparseSdf3_Initialize,
	AlignedObjectArray_btSoftBody_MaterialPtr_at = CLIB.btAlignedObjectArray_btSoftBody_MaterialPtr_at,
	SoftBody_Tetra_getC2 = CLIB.btSoftBody_Tetra_getC2,
	DispatcherInfo_getDispatchFunc = CLIB.btDispatcherInfo_getDispatchFunc,
	DiscreteCollisionDetectorInterface_ClosestPointInput_delete = CLIB.btDiscreteCollisionDetectorInterface_ClosestPointInput_delete,
	CollisionWorld_addCollisionObject = CLIB.btCollisionWorld_addCollisionObject,
	RotationalLimitMotor2_getStopERP = CLIB.btRotationalLimitMotor2_getStopERP,
	CompoundCompoundCollisionAlgorithm_SwappedCreateFunc_new = CLIB.btCompoundCompoundCollisionAlgorithm_SwappedCreateFunc_new,
	CollisionWorld_LocalConvexResult_getHitNormalLocal = CLIB.btCollisionWorld_LocalConvexResult_getHitNormalLocal,
	Convex2dConvex2dAlgorithm_CreateFunc_getPdSolver = CLIB.btConvex2dConvex2dAlgorithm_CreateFunc_getPdSolver,
	CollisionWorld_AllHitsRayResultCallback_new = CLIB.btCollisionWorld_AllHitsRayResultCallback_new,
	DbvtBroadphase_performDeferredRemoval = CLIB.btDbvtBroadphase_performDeferredRemoval,
	BvhTriangleMeshShape_setTriangleInfoMap = CLIB.btBvhTriangleMeshShape_setTriangleInfoMap,
	Generic6DofSpringConstraint_setEquilibriumPoint2 = CLIB.btGeneric6DofSpringConstraint_setEquilibriumPoint2,
	ConvexPlaneCollisionAlgorithm_collideSingleContact = CLIB.btConvexPlaneCollisionAlgorithm_collideSingleContact,
	AlignedObjectArray_btSoftBody_MaterialPtr_push_back = CLIB.btAlignedObjectArray_btSoftBody_MaterialPtr_push_back,
	CollisionObject_setCustomDebugColor = CLIB.btCollisionObject_setCustomDebugColor,
	CapsuleShapeZ_new = CLIB.btCapsuleShapeZ_new,
	GImpactShapeInterface_updateBound = CLIB.btGImpactShapeInterface_updateBound,
	TranslationalLimitMotor2_getStopERP = CLIB.btTranslationalLimitMotor2_getStopERP,
	HingeConstraint_enableMotor = CLIB.btHingeConstraint_enableMotor,
	QuantizedBvhNode_getQuantizedAabbMax = CLIB.btQuantizedBvhNode_getQuantizedAabbMax,
	CollisionWorld_RayResultCallback_getCollisionFilterGroup = CLIB.btCollisionWorld_RayResultCallback_getCollisionFilterGroup,
	GjkEpaPenetrationDepthSolver_new = CLIB.btGjkEpaPenetrationDepthSolver_new,
	CollisionObject_getActivationState = CLIB.btCollisionObject_getActivationState,
	SoftRigidCollisionAlgorithm_CreateFunc_new = CLIB.btSoftRigidCollisionAlgorithm_CreateFunc_new,
	GjkPairDetector_getCachedSeparatingDistance = CLIB.btGjkPairDetector_getCachedSeparatingDistance,
	CollisionAlgorithmCreateFunc_setSwapped = CLIB.btCollisionAlgorithmCreateFunc_setSwapped,
	MultiBody_setupPlanar = CLIB.btMultiBody_setupPlanar,
	CollisionWorld_RayResultCallback_needsCollision = CLIB.btCollisionWorld_RayResultCallback_needsCollision,
	CollisionObject_mergesSimulationIslands = CLIB.btCollisionObject_mergesSimulationIslands,
	PersistentManifold_removeContactPoint = CLIB.btPersistentManifold_removeContactPoint,
	CollisionWorld_ContactResultCallback_getClosestDistanceThreshold = CLIB.btCollisionWorld_ContactResultCallback_getClosestDistanceThreshold,
	ConvexCast_CastResult_setNormal = CLIB.btConvexCast_CastResult_setNormal,
	ConstraintSolver_getSolverType = CLIB.btConstraintSolver_getSolverType,
	ConvexPolyhedron_getRadius = CLIB.btConvexPolyhedron_getRadius,
	ManifoldPoint_getContactCFM = CLIB.btManifoldPoint_getContactCFM,
	GImpactBvh_isTrimesh = CLIB.btGImpactBvh_isTrimesh,
	MultiBody_getUserPointer = CLIB.btMultiBody_getUserPointer,
	HingeConstraint_setMotorTargetVelocity = CLIB.btHingeConstraint_setMotorTargetVelocity,
	DbvtBroadphase_getPid = CLIB.btDbvtBroadphase_getPid,
	AxisSweep3_addHandle = CLIB.btAxisSweep3_addHandle,
	BroadphaseProxy_getCollisionFilterMask = CLIB.btBroadphaseProxy_getCollisionFilterMask,
	ConvexPenetrationDepthSolver_delete = CLIB.btConvexPenetrationDepthSolver_delete,
	SoftBodyWorldInfo_getBroadphase = CLIB.btSoftBodyWorldInfo_getBroadphase,
	ManifoldPoint_setNormalWorldOnB = CLIB.btManifoldPoint_setNormalWorldOnB,
	TypedConstraint_solveConstraintObsolete = CLIB.btTypedConstraint_solveConstraintObsolete,
	TypedConstraint_getFixedBody = CLIB.btTypedConstraint_getFixedBody,
	DiscreteDynamicsWorld_getApplySpeculativeContactRestitution = CLIB.btDiscreteDynamicsWorld_getApplySpeculativeContactRestitution,
	TranslationalLimitMotor_setMaxMotorForce = CLIB.btTranslationalLimitMotor_setMaxMotorForce,
	AABB_overlapping_trans_conservative = CLIB.btAABB_overlapping_trans_conservative,
	SoftBodyRigidBodyCollisionConfiguration_new = CLIB.btSoftBodyRigidBodyCollisionConfiguration_new,
	BvhTriangleMeshShape_new = CLIB.btBvhTriangleMeshShape_new,
	GImpactMeshShapePart_TrimeshPrimitiveManager_setStride = CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_setStride,
	AABB_find_intersection = CLIB.btAABB_find_intersection,
	MultiBodyConstraint_allocateJacobiansMultiDof = CLIB.btMultiBodyConstraint_allocateJacobiansMultiDof,
	CollisionDispatcher_setNearCallback = CLIB.btCollisionDispatcher_setNearCallback,
	ContactSolverInfoData_setTimeStep = CLIB.btContactSolverInfoData_setTimeStep,
	CompoundShape_new = CLIB.btCompoundShape_new,
	CollisionObjectWrapper_getCollisionObject = CLIB.btCollisionObjectWrapper_getCollisionObject,
	TranslationalLimitMotor_getTargetVelocity = CLIB.btTranslationalLimitMotor_getTargetVelocity,
	ConeTwistConstraint_getBFrame = CLIB.btConeTwistConstraint_getBFrame,
	Face_new = CLIB.btFace_new,
	TranslationalLimitMotor_getMaxMotorForce = CLIB.btTranslationalLimitMotor_getMaxMotorForce,
	SoftBody_RayFromToCaster_getRayNormalizedDirection = CLIB.btSoftBody_RayFromToCaster_getRayNormalizedDirection,
	ContactSolverInfoData_setSplitImpulse = CLIB.btContactSolverInfoData_setSplitImpulse,
	BvhTriangleMeshShape_setOptimizedBvh2 = CLIB.btBvhTriangleMeshShape_setOptimizedBvh2,
	CollisionObject_getInterpolationWorldTransform = CLIB.btCollisionObject_getInterpolationWorldTransform,
	IDebugDrawWrapper_getGCHandle = CLIB.btIDebugDrawWrapper_getGCHandle,
	BroadphaseProxy_setCollisionFilterMask = CLIB.btBroadphaseProxy_setCollisionFilterMask,
	CollisionObject_getCustomDebugColor = CLIB.btCollisionObject_getCustomDebugColor,
	RotationalLimitMotor_setMaxLimitForce = CLIB.btRotationalLimitMotor_setMaxLimitForce,
	GearConstraint_setAxisB = CLIB.btGearConstraint_setAxisB,
	RotationalLimitMotor_setCurrentLimit = CLIB.btRotationalLimitMotor_setCurrentLimit,
	Generic6DofSpring2Constraint_matrixToEulerYZX = CLIB.btGeneric6DofSpring2Constraint_matrixToEulerYZX,
	CollisionObject_getIslandTag = CLIB.btCollisionObject_getIslandTag,
	Generic6DofConstraint_getFlags = CLIB.btGeneric6DofConstraint_getFlags,
	CollisionObject_getRestitution = CLIB.btCollisionObject_getRestitution,
	CollisionAlgorithmCreateFunc_getSwapped = CLIB.btCollisionAlgorithmCreateFunc_getSwapped,
	Generic6DofConstraint_getUseSolveConstraintObsolete = CLIB.btGeneric6DofConstraint_getUseSolveConstraintObsolete,
	CollisionObject_getUserIndex2 = CLIB.btCollisionObject_getUserIndex2,
	VoronoiSimplexSolver_getCachedP2 = CLIB.btVoronoiSimplexSolver_getCachedP2,
	RigidBody_setNewBroadphaseProxy = CLIB.btRigidBody_setNewBroadphaseProxy,
	TypedConstraint_enableFeedback = CLIB.btTypedConstraint_enableFeedback,
	MultiSphereShape_new = CLIB.btMultiSphereShape_new,
	CollisionWorld_serialize = CLIB.btCollisionWorld_serialize,
	AABB_new5 = CLIB.btAABB_new5,
	CollisionObject_getUserPointer = CLIB.btCollisionObject_getUserPointer,
	GImpactQuantizedBvh_buildSet = CLIB.btGImpactQuantizedBvh_buildSet,
	RotationalLimitMotor_getTargetVelocity = CLIB.btRotationalLimitMotor_getTargetVelocity,
	GImpactMeshShapePart_TrimeshPrimitiveManager_getStride = CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_getStride,
	CollisionObjectWrapper_getPartId = CLIB.btCollisionObjectWrapper_getPartId,
	CompoundShape_createAabbTreeFromChildren = CLIB.btCompoundShape_createAabbTreeFromChildren,
	OverlapFilterCallback_needBroadphaseCollision = CLIB.btOverlapFilterCallback_needBroadphaseCollision,
	ConstraintSetting_getDamping = CLIB.btConstraintSetting_getDamping,
	TranslationalLimitMotor2_setMotorCFM = CLIB.btTranslationalLimitMotor2_setMotorCFM,
	Face_delete = CLIB.btFace_delete,
	RotationalLimitMotor2_getCurrentLimitError = CLIB.btRotationalLimitMotor2_getCurrentLimitError,
	BroadphaseAabbCallback_process = CLIB.btBroadphaseAabbCallback_process,
	CollisionWorld_ClosestConvexResultCallback_setHitNormalWorld = CLIB.btCollisionWorld_ClosestConvexResultCallback_setHitNormalWorld,
	CollisionObject_getFriction = CLIB.btCollisionObject_getFriction,
	Dbvt_sStkCLN_getNode = CLIB.btDbvt_sStkCLN_getNode,
	HingeConstraint_getLimitSign = CLIB.btHingeConstraint_getLimitSign,
	UsageBitfield_reset = CLIB.btUsageBitfield_reset,
	MultiBodyLinkCollider_getLink = CLIB.btMultiBodyLinkCollider_getLink,
	CollisionWorld_RayResultCallback_hasHit = CLIB.btCollisionWorld_RayResultCallback_hasHit,
	CollisionShape_setUserIndex = CLIB.btCollisionShape_setUserIndex,
	AlignedObjectArray_btSoftBody_ClusterPtr_at = CLIB.btAlignedObjectArray_btSoftBody_ClusterPtr_at,
	ConvexPolyhedron_getMC = CLIB.btConvexPolyhedron_getMC,
	Generic6DofSpring2Constraint_matrixToEulerXYZ = CLIB.btGeneric6DofSpring2Constraint_matrixToEulerXYZ,
	DispatcherInfo_setUseEpa = CLIB.btDispatcherInfo_setUseEpa,
	ConvexShape_localGetSupportVertexNonVirtual = CLIB.btConvexShape_localGetSupportVertexNonVirtual,
	DefaultCollisionConstructionInfo_getUseEpaPenetrationAlgorithm = CLIB.btDefaultCollisionConstructionInfo_getUseEpaPenetrationAlgorithm,
	BoxBoxDetector_setBox2 = CLIB.btBoxBoxDetector_setBox2,
	GImpactMeshShapePart_TrimeshPrimitiveManager_unlock = CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_unlock,
	ConvexPolyhedron_setRadius = CLIB.btConvexPolyhedron_setRadius,
	Dbvt_benchmark = CLIB.btDbvt_benchmark,
	ConvexPolyhedron_delete = CLIB.btConvexPolyhedron_delete,
	Hinge2Constraint_getAngle1 = CLIB.btHinge2Constraint_getAngle1,
	CompoundShapeChild_getChildMargin = CLIB.btCompoundShapeChild_getChildMargin,
	SortedOverlappingPairCache_needsBroadphaseCollision = CLIB.btSortedOverlappingPairCache_needsBroadphaseCollision,
	CylinderShapeZ_new2 = CLIB.btCylinderShapeZ_new2,
	CollisionWorld_ClosestConvexResultCallback_setHitPointWorld = CLIB.btCollisionWorld_ClosestConvexResultCallback_setHitPointWorld,
	CollisionWorld_ClosestRayResultCallback_getHitNormalWorld = CLIB.btCollisionWorld_ClosestRayResultCallback_getHitNormalWorld,
	GImpactShapeInterface_getBulletTetrahedron = CLIB.btGImpactShapeInterface_getBulletTetrahedron,
	ActionInterface_debugDraw = CLIB.btActionInterface_debugDraw,
	Generic6DofSpring2Constraint_setAngularLowerLimit = CLIB.btGeneric6DofSpring2Constraint_setAngularLowerLimit,
	CollisionWorld_LocalRayResult_setHitNormalLocal = CLIB.btCollisionWorld_LocalRayResult_setHitNormalLocal,
	Dbvt_ICollide_Process3 = CLIB.btDbvt_ICollide_Process3,
	CollisionObject_setUserIndex2 = CLIB.btCollisionObject_setUserIndex2,
	AlignedObjectArray_btPersistentManifoldPtr_resizeNoInitialize = CLIB.btAlignedObjectArray_btPersistentManifoldPtr_resizeNoInitialize,
	CollisionWorld_addCollisionObject3 = CLIB.btCollisionWorld_addCollisionObject3,
	CollisionWorld_LocalShapeInfo_getTriangleIndex = CLIB.btCollisionWorld_LocalShapeInfo_getTriangleIndex,
	StorageResult_getDistance = CLIB.btStorageResult_getDistance,
	GImpactMeshShapePart_TrimeshPrimitiveManager_setType = CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_setType,
	BoxShape_getHalfExtentsWithMargin = CLIB.btBoxShape_getHalfExtentsWithMargin,
	ConvexPlaneCollisionAlgorithm_new = CLIB.btConvexPlaneCollisionAlgorithm_new,
	DiscreteCollisionDetectorInterface_Result_setShapeIdentifiersA = CLIB.btDiscreteCollisionDetectorInterface_Result_setShapeIdentifiersA,
	DiscreteDynamicsWorld_setSynchronizeAllMotionStates = CLIB.btDiscreteDynamicsWorld_setSynchronizeAllMotionStates,
	CollisionWorld_ClosestRayResultCallback_setHitPointWorld = CLIB.btCollisionWorld_ClosestRayResultCallback_setHitPointWorld,
	CompoundShapeChild_setTransform = CLIB.btCompoundShapeChild_setTransform,
	SoftBody_Cluster_getAv = CLIB.btSoftBody_Cluster_getAv,
	DefaultCollisionConstructionInfo_getCollisionAlgorithmPool = CLIB.btDefaultCollisionConstructionInfo_getCollisionAlgorithmPool,
	JointFeedback_getAppliedForceBodyB = CLIB.btJointFeedback_getAppliedForceBodyB,
	TranslationalLimitMotor2_setCurrentLinearDiff = CLIB.btTranslationalLimitMotor2_setCurrentLinearDiff,
	CollisionAlgorithm_delete = CLIB.btCollisionAlgorithm_delete,
	CollisionWorld_LocalRayResult_setCollisionObject = CLIB.btCollisionWorld_LocalRayResult_setCollisionObject,
	DbvtBroadphase_setDupdates = CLIB.btDbvtBroadphase_setDupdates,
	DbvtAabbMm_FromMM = CLIB.btDbvtAabbMm_FromMM,
	CollisionWorld_ContactResultCallback_setCollisionFilterMask = CLIB.btCollisionWorld_ContactResultCallback_setCollisionFilterMask,
	Box2dShape_new = CLIB.btBox2dShape_new,
	DiscreteCollisionDetectorInterface_ClosestPointInput_setTransformB = CLIB.btDiscreteCollisionDetectorInterface_ClosestPointInput_setTransformB,
	ConeTwistConstraint_getAngularOnly = CLIB.btConeTwistConstraint_getAngularOnly,
	CollisionObject_setRestitution = CLIB.btCollisionObject_setRestitution,
	CollisionAlgorithmConstructionInfo_new2 = CLIB.btCollisionAlgorithmConstructionInfo_new2,
	HingeAccumulatedAngleConstraint_new4 = CLIB.btHingeAccumulatedAngleConstraint_new4,
	ContactSolverInfoData_setNumIterations = CLIB.btContactSolverInfoData_setNumIterations,
	BvhTree_getLeftNode = CLIB.btBvhTree_getLeftNode,
	Generic6DofConstraint_getUseLinearReferenceFrameA = CLIB.btGeneric6DofConstraint_getUseLinearReferenceFrameA,
	StorageResult_setDistance = CLIB.btStorageResult_setDistance,
	Generic6DofSpring2Constraint_getCalculatedTransformB = CLIB.btGeneric6DofSpring2Constraint_getCalculatedTransformB,
	CollisionWorld_ContactResultCallback_delete = CLIB.btCollisionWorld_ContactResultCallback_delete,
	ConvexTriangleMeshShape_calculatePrincipalAxisTransform = CLIB.btConvexTriangleMeshShape_calculatePrincipalAxisTransform,
	DbvtBroadphase_setDeferedcollide = CLIB.btDbvtBroadphase_setDeferedcollide,
	DiscreteDynamicsWorld_setLatencyMotionStateInterpolation = CLIB.btDiscreteDynamicsWorld_setLatencyMotionStateInterpolation,
	Generic6DofConstraint_isLimited = CLIB.btGeneric6DofConstraint_isLimited,
	DispatcherInfo_getConvexConservativeDistanceThreshold = CLIB.btDispatcherInfo_getConvexConservativeDistanceThreshold,
	DispatcherInfo_getUseConvexConservativeDistanceUtil = CLIB.btDispatcherInfo_getUseConvexConservativeDistanceUtil,
	DispatcherInfo_setConvexConservativeDistanceThreshold = CLIB.btDispatcherInfo_setConvexConservativeDistanceThreshold,
	GImpactBvh_getRightNode = CLIB.btGImpactBvh_getRightNode,
	CollisionWorld_ConvexResultCallback_setCollisionFilterGroup = CLIB.btCollisionWorld_ConvexResultCallback_setCollisionFilterGroup,
	DispatcherInfo_setEnableSPU = CLIB.btDispatcherInfo_setEnableSPU,
	MultiBody_setBaseName = CLIB.btMultiBody_setBaseName,
	Generic6DofSpring2Constraint_getFrameOffsetB = CLIB.btGeneric6DofSpring2Constraint_getFrameOffsetB,
	CollisionWorld_LocalShapeInfo_delete = CLIB.btCollisionWorld_LocalShapeInfo_delete,
	DispatcherInfo_setUseConvexConservativeDistanceUtil = CLIB.btDispatcherInfo_setUseConvexConservativeDistanceUtil,
	TriangleMesh_setWeldingThreshold = CLIB.btTriangleMesh_setWeldingThreshold,
	DbvtNode_getDataAsInt = CLIB.btDbvtNode_getDataAsInt,
	ConvexConvexAlgorithm_CreateFunc_setNumPerturbationIterations = CLIB.btConvexConvexAlgorithm_CreateFunc_setNumPerturbationIterations,
	MLCPSolver_new = CLIB.btMLCPSolver_new,
	SoftBody_AJoint_IControlWrapper_getWrapperData = CLIB.btSoftBody_AJoint_IControlWrapper_getWrapperData,
	BvhTriangleMeshShape_partialRefitTree = CLIB.btBvhTriangleMeshShape_partialRefitTree,
	Dbvt_update6 = CLIB.btDbvt_update6,
	Dispatcher_needsResponse = CLIB.btDispatcher_needsResponse,
	DynamicsWorld_addRigidBody = CLIB.btDynamicsWorld_addRigidBody,
	SoftBody_Impulse_getVelocity = CLIB.btSoftBody_Impulse_getVelocity,
	DynamicsWorld_getConstraint = CLIB.btDynamicsWorld_getConstraint,
	DynamicsWorld_getConstraintSolver = CLIB.btDynamicsWorld_getConstraintSolver,
	DynamicsWorld_getNumConstraints = CLIB.btDynamicsWorld_getNumConstraints,
	MultiBodyJointMotor_setVelocityTarget2 = CLIB.btMultiBodyJointMotor_setVelocityTarget2,
	DynamicsWorld_removeRigidBody = CLIB.btDynamicsWorld_removeRigidBody,
	CollisionObjectWrapper_getCollisionShape = CLIB.btCollisionObjectWrapper_getCollisionShape,
	ConeTwistConstraint_getFlags = CLIB.btConeTwistConstraint_getFlags,
	Dbvt_optimizeTopDown2 = CLIB.btDbvt_optimizeTopDown2,
	SliderConstraint_getFrameOffsetB = CLIB.btSliderConstraint_getFrameOffsetB,
	CollisionWorld_performDiscreteCollisionDetection = CLIB.btCollisionWorld_performDiscreteCollisionDetection,
	RotationalLimitMotor2_setCurrentLimitError = CLIB.btRotationalLimitMotor2_setCurrentLimitError,
	MultiBody_getBaseForce = CLIB.btMultiBody_getBaseForce,
	GearConstraint_getAxisB = CLIB.btGearConstraint_getAxisB,
	CollisionShape_calculateSerializeBufferSize = CLIB.btCollisionShape_calculateSerializeBufferSize,
	MultiBodyConstraint_createConstraintRows = CLIB.btMultiBodyConstraint_createConstraintRows,
	ConvexShape_localGetSupportingVertex = CLIB.btConvexShape_localGetSupportingVertex,
	GearConstraint_setRatio = CLIB.btGearConstraint_setRatio,
	GImpactMeshShapePart_TrimeshPrimitiveManager_get_bullet_triangle = CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_get_bullet_triangle,
	DbvtBroadphase_getDupdates = CLIB.btDbvtBroadphase_getDupdates,
	RotationalLimitMotor_getCurrentLimit = CLIB.btRotationalLimitMotor_getCurrentLimit,
	SphereShape_getRadius = CLIB.btSphereShape_getRadius,
	RotationalLimitMotor_setTargetVelocity = CLIB.btRotationalLimitMotor_setTargetVelocity,
	RotationalLimitMotor2_getMaxMotorForce = CLIB.btRotationalLimitMotor2_getMaxMotorForce,
	MinkowskiSumShape_getShapeB = CLIB.btMinkowskiSumShape_getShapeB,
	BroadphaseProxy_getUid = CLIB.btBroadphaseProxy_getUid,
	RotationalLimitMotor_getLimitSoftness = CLIB.btRotationalLimitMotor_getLimitSoftness,
	ConeTwistConstraint_getMaxMotorImpulse = CLIB.btConeTwistConstraint_getMaxMotorImpulse,
	RotationalLimitMotor_getLoLimit = CLIB.btRotationalLimitMotor_getLoLimit,
	RotationalLimitMotor_needApplyTorques = CLIB.btRotationalLimitMotor_needApplyTorques,
	RotationalLimitMotor_solveAngularLimits = CLIB.btRotationalLimitMotor_solveAngularLimits,
	CollisionObject_setInterpolationLinearVelocity = CLIB.btCollisionObject_setInterpolationLinearVelocity,
	DbvtBroadphase_getFupdates = CLIB.btDbvtBroadphase_getFupdates,
	RotationalLimitMotor_setHiLimit = CLIB.btRotationalLimitMotor_setHiLimit,
	GImpactCompoundShape_CompoundPrimitiveManager_getCompoundShape = CLIB.btGImpactCompoundShape_CompoundPrimitiveManager_getCompoundShape,
	MultibodyLink_setInertiaLocal = CLIB.btMultibodyLink_setInertiaLocal,
	CollisionWorld_RayResultCallback_addSingleResult = CLIB.btCollisionWorld_RayResultCallback_addSingleResult,
	RotationalLimitMotor_setEnableMotor = CLIB.btRotationalLimitMotor_setEnableMotor,
	ConvexCast_CastResult_getFraction = CLIB.btConvexCast_CastResult_getFraction,
	RotationalLimitMotor_setLoLimit = CLIB.btRotationalLimitMotor_setLoLimit,
	GImpactQuantizedBvh_new2 = CLIB.btGImpactQuantizedBvh_new2,
	RotationalLimitMotor_setMaxMotorForce = CLIB.btRotationalLimitMotor_setMaxMotorForce,
	RotationalLimitMotor_setStopCFM = CLIB.btRotationalLimitMotor_setStopCFM,
	ContactSolverInfoData_getErp = CLIB.btContactSolverInfoData_getErp,
	CollisionWorld_LocalRayResult_setLocalShapeInfo = CLIB.btCollisionWorld_LocalRayResult_setLocalShapeInfo,
	StorageResult_setClosestPointInB = CLIB.btStorageResult_setClosestPointInB,
	StorageResult_getClosestPointInB = CLIB.btStorageResult_getClosestPointInB,
	TranslationalLimitMotor_getCurrentLimit = CLIB.btTranslationalLimitMotor_getCurrentLimit,
	MultiBodyDynamicsWorld_getNumMultibodies = CLIB.btMultiBodyDynamicsWorld_getNumMultibodies,
	Dbvt_write = CLIB.btDbvt_write,
	TranslationalLimitMotor_getNormalCFM = CLIB.btTranslationalLimitMotor_getNormalCFM,
	GImpactCompoundShape_CompoundPrimitiveManager_new2 = CLIB.btGImpactCompoundShape_CompoundPrimitiveManager_new2,
	GImpactBvh_rayQuery = CLIB.btGImpactBvh_rayQuery,
	HingeConstraint_getMaxMotorImpulse = CLIB.btHingeConstraint_getMaxMotorImpulse,
	TranslationalLimitMotor_setCurrentLimitError = CLIB.btTranslationalLimitMotor_setCurrentLimitError,
	TranslationalLimitMotor_setCurrentLinearDiff = CLIB.btTranslationalLimitMotor_setCurrentLinearDiff,
	TriangleBuffer_clearBuffer = CLIB.btTriangleBuffer_clearBuffer,
	TranslationalLimitMotor_setLimitSoftness = CLIB.btTranslationalLimitMotor_setLimitSoftness,
	TranslationalLimitMotor_setLowerLimit = CLIB.btTranslationalLimitMotor_setLowerLimit,
	MinkowskiSumShape_new = CLIB.btMinkowskiSumShape_new,
	CollisionWorld_getForceUpdateAllAabbs = CLIB.btCollisionWorld_getForceUpdateAllAabbs,
	MultiBodySolverConstraint_getDeltaVelAindex = CLIB.btMultiBodySolverConstraint_getDeltaVelAindex,
	TranslationalLimitMotor_setStopCFM = CLIB.btTranslationalLimitMotor_setStopCFM,
	PersistentManifold_refreshContactPoints = CLIB.btPersistentManifold_refreshContactPoints,
	DbvtAabbMm_ProjectMinimum = CLIB.btDbvtAabbMm_ProjectMinimum,
	HingeConstraint_new4 = CLIB.btHingeConstraint_new4,
	MultiBody_getMaxAppliedImpulse = CLIB.btMultiBody_getMaxAppliedImpulse,
	MultiBody_isUsingRK4Integration = CLIB.btMultiBody_isUsingRK4Integration,
	MultibodyLink_getAbsFrameLocVelocity = CLIB.btMultibodyLink_getAbsFrameLocVelocity,
	CollisionWorld_LocalConvexResult_getHitPointLocal = CLIB.btCollisionWorld_LocalConvexResult_getHitPointLocal,
	TypedConstraint_internalGetAppliedImpulse = CLIB.btTypedConstraint_internalGetAppliedImpulse,
	SoftBodyRigidBodyCollisionConfiguration_new2 = CLIB.btSoftBodyRigidBodyCollisionConfiguration_new2,
	Generic6DofSpring2Constraint_setBounce = CLIB.btGeneric6DofSpring2Constraint_setBounce,
	Generic6DofSpringConstraint_setEquilibriumPoint3 = CLIB.btGeneric6DofSpringConstraint_setEquilibriumPoint3,
	CompoundShape_getChildShape = CLIB.btCompoundShape_getChildShape,
	MultiBodyLinkCollider_upcast = CLIB.btMultiBodyLinkCollider_upcast,
	GImpactMeshShapePart_TrimeshPrimitiveManager_new2 = CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_new2,
	PairCachingGhostObject_getOverlappingPairCache = CLIB.btPairCachingGhostObject_getOverlappingPairCache,
	TriangleMesh_addTriangle = CLIB.btTriangleMesh_addTriangle,
	SubSimplexClosestResult_getDegenerate = CLIB.btSubSimplexClosestResult_getDegenerate,
	DantzigSolver_new = CLIB.btDantzigSolver_new,
	Generic6DofConstraint_getLinearUpperLimit = CLIB.btGeneric6DofConstraint_getLinearUpperLimit,
	Generic6DofConstraint_getUseFrameOffset = CLIB.btGeneric6DofConstraint_getUseFrameOffset,
	CollisionWorld_getBroadphase = CLIB.btCollisionWorld_getBroadphase,
	CollisionWorld_ClosestRayResultCallback_getRayToWorld = CLIB.btCollisionWorld_ClosestRayResultCallback_getRayToWorld,
	GImpactBvh_find_collision = CLIB.btGImpactBvh_find_collision,
	Generic6DofConstraint_setLinearLowerLimit = CLIB.btGeneric6DofConstraint_setLinearLowerLimit,
	UniversalConstraint_getAngle2 = CLIB.btUniversalConstraint_getAngle2,
	RotationalLimitMotor2_getEnableMotor = CLIB.btRotationalLimitMotor2_getEnableMotor,
	VoronoiSimplexSolver_closest = CLIB.btVoronoiSimplexSolver_closest,
	TranslationalLimitMotor2_getCurrentLimitErrorHi = CLIB.btTranslationalLimitMotor2_getCurrentLimitErrorHi,
	PersistentManifold_addManifoldPoint = CLIB.btPersistentManifold_addManifoldPoint,
	AlignedObjectArray_btSoftBody_Node_at = CLIB.btAlignedObjectArray_btSoftBody_Node_at,
	CollisionDispatcher_setCollisionConfiguration = CLIB.btCollisionDispatcher_setCollisionConfiguration,
	ContactSolverInfoData_getGlobalCfm = CLIB.btContactSolverInfoData_getGlobalCfm,
	RotationalLimitMotor2_setBounce = CLIB.btRotationalLimitMotor2_setBounce,
	RotationalLimitMotor2_setCurrentLimit = CLIB.btRotationalLimitMotor2_setCurrentLimit,
	RotationalLimitMotor2_setCurrentLimitErrorHi = CLIB.btRotationalLimitMotor2_setCurrentLimitErrorHi,
	GImpactMeshShapePart_TrimeshPrimitiveManager_getVertexbase = CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_getVertexbase,
	RotationalLimitMotor2_setEnableMotor = CLIB.btRotationalLimitMotor2_setEnableMotor,
	MultiBodyDynamicsWorld_getMultiBody = CLIB.btMultiBodyDynamicsWorld_getMultiBody,
	RotationalLimitMotor2_setHiLimit = CLIB.btRotationalLimitMotor2_setHiLimit,
	MultiBodySolverConstraint_getSolverBodyIdB = CLIB.btMultiBodySolverConstraint_getSolverBodyIdB,
	RotationalLimitMotor2_setMotorCFM = CLIB.btRotationalLimitMotor2_setMotorCFM,
	RotationalLimitMotor2_setServoTarget = CLIB.btRotationalLimitMotor2_setServoTarget,
	MultiBody_worldDirToLocal = CLIB.btMultiBody_worldDirToLocal,
	CollisionWorld_ContactResultCallbackWrapper_needsCollision = CLIB.btCollisionWorld_ContactResultCallbackWrapper_needsCollision,
	RotationalLimitMotor2_setTargetVelocity = CLIB.btRotationalLimitMotor2_setTargetVelocity,
	BroadphaseRayCallback_setRayDirectionInverse = CLIB.btBroadphaseRayCallback_setRayDirectionInverse,
	DefaultSoftBodySolver_copySoftBodyToVertexBuffer = CLIB.btDefaultSoftBodySolver_copySoftBodyToVertexBuffer,
	Generic6DofSpringConstraint_getEquilibriumPoint = CLIB.btGeneric6DofSpringConstraint_getEquilibriumPoint,
	MultiBody_setLinearDamping = CLIB.btMultiBody_setLinearDamping,
	TranslationalLimitMotor2_getCurrentLinearDiff = CLIB.btTranslationalLimitMotor2_getCurrentLinearDiff,
	ConeTwistConstraint_getLimitSoftness = CLIB.btConeTwistConstraint_getLimitSoftness,
	GImpactQuantizedBvh_isTrimesh = CLIB.btGImpactQuantizedBvh_isTrimesh,
	SoftBody_clusterVelocity = CLIB.btSoftBody_clusterVelocity,
	MultiBody_setAngularDamping = CLIB.btMultiBody_setAngularDamping,
	DbvtBroadphase_getStageRoots = CLIB.btDbvtBroadphase_getStageRoots,
	CollisionWorld_ConvexResultCallbackWrapper_new = CLIB.btCollisionWorld_ConvexResultCallbackWrapper_new,
	SoftBody_Body_angularVelocity = CLIB.btSoftBody_Body_angularVelocity,
	Hinge2Constraint_setUpperLimit = CLIB.btHinge2Constraint_setUpperLimit,
	ShapeHull_numTriangles = CLIB.btShapeHull_numTriangles,
	TranslationalLimitMotor2_getUpperLimit = CLIB.btTranslationalLimitMotor2_getUpperLimit,
	VoronoiSimplexSolver_getNumVertices = CLIB.btVoronoiSimplexSolver_getNumVertices,
	GImpactMeshShapePart_TrimeshPrimitiveManager_setLock_count = CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_setLock_count,
	DbvtBroadphase_setNewpairs = CLIB.btDbvtBroadphase_setNewpairs,
	PersistentManifold_setBodies = CLIB.btPersistentManifold_setBodies,
	TranslationalLimitMotor2_setLowerLimit = CLIB.btTranslationalLimitMotor2_setLowerLimit,
	TranslationalLimitMotor2_setMaxMotorForce = CLIB.btTranslationalLimitMotor2_setMaxMotorForce,
	TranslationalLimitMotor2_setMotorERP = CLIB.btTranslationalLimitMotor2_setMotorERP,
	TranslationalLimitMotor2_setSpringDamping = CLIB.btTranslationalLimitMotor2_setSpringDamping,
	TranslationalLimitMotor2_setSpringStiffness = CLIB.btTranslationalLimitMotor2_setSpringStiffness,
	TranslationalLimitMotor2_setStopCFM = CLIB.btTranslationalLimitMotor2_setStopCFM,
	TranslationalLimitMotor2_setTargetVelocity = CLIB.btTranslationalLimitMotor2_setTargetVelocity,
	DefaultCollisionConstructionInfo_setDefaultMaxPersistentManifoldPoolSize = CLIB.btDefaultCollisionConstructionInfo_setDefaultMaxPersistentManifoldPoolSize,
	SerializerWrapper_new = CLIB.btSerializerWrapper_new,
	MultiBody_forwardKinematics = CLIB.btMultiBody_forwardKinematics,
	CollisionWorld_debugDrawWorld = CLIB.btCollisionWorld_debugDrawWorld,
	CollisionObject_hasAnisotropicFriction = CLIB.btCollisionObject_hasAnisotropicFriction,
	Generic6DofSpring2Constraint_getAngle = CLIB.btGeneric6DofSpring2Constraint_getAngle,
	Generic6DofSpring2Constraint_getAngularLowerLimitReversed = CLIB.btGeneric6DofSpring2Constraint_getAngularLowerLimitReversed,
	Generic6DofSpring2Constraint_getAngularUpperLimit = CLIB.btGeneric6DofSpring2Constraint_getAngularUpperLimit,
	Generic6DofSpring2Constraint_getAngularUpperLimitReversed = CLIB.btGeneric6DofSpring2Constraint_getAngularUpperLimitReversed,
	ManifoldPoint_setDistance1 = CLIB.btManifoldPoint_setDistance1,
	MultiBodyConstraint_setMaxAppliedImpulse = CLIB.btMultiBodyConstraint_setMaxAppliedImpulse,
	SparseSdf_new = CLIB.btSparseSdf_new,
	Generic6DofSpring2Constraint_setEquilibriumPoint = CLIB.btGeneric6DofSpring2Constraint_setEquilibriumPoint,
	Generic6DofSpring2Constraint_getLinearUpperLimit = CLIB.btGeneric6DofSpring2Constraint_getLinearUpperLimit,
	ManifoldPoint_getPartId0 = CLIB.btManifoldPoint_getPartId0,
	CollisionObject_setCcdMotionThreshold = CLIB.btCollisionObject_setCcdMotionThreshold,
	SoftBody_addAeroForceToFace = CLIB.btSoftBody_addAeroForceToFace,
	Generic6DofSpring2Constraint_setAngularUpperLimit = CLIB.btGeneric6DofSpring2Constraint_setAngularUpperLimit,
	Generic6DofSpring2Constraint_setAngularUpperLimitReversed = CLIB.btGeneric6DofSpring2Constraint_setAngularUpperLimitReversed,
	CylinderShape_getUpAxis = CLIB.btCylinderShape_getUpAxis,
	RotationalLimitMotor2_setSpringStiffness = CLIB.btRotationalLimitMotor2_setSpringStiffness,
	Generic6DofSpring2Constraint_setEquilibriumPoint3 = CLIB.btGeneric6DofSpring2Constraint_setEquilibriumPoint3,
	BvhTree_get_node_pointer2 = CLIB.btBvhTree_get_node_pointer2,
	Generic6DofSpring2Constraint_setLinearLowerLimit = CLIB.btGeneric6DofSpring2Constraint_setLinearLowerLimit,
	Triangle_new = CLIB.btTriangle_new,
	GImpactQuantizedBvh_find_collision = CLIB.btGImpactQuantizedBvh_find_collision,
	CollisionObject_getCcdMotionThreshold = CLIB.btCollisionObject_getCcdMotionThreshold,
	StaticPlaneShape_new = CLIB.btStaticPlaneShape_new,
	ManifoldPoint_getCombinedContactStiffness1 = CLIB.btManifoldPoint_getCombinedContactStiffness1,
	SoftBody_Cluster_getFramerefs = CLIB.btSoftBody_Cluster_getFramerefs,
	GImpactCompoundShape_CompoundPrimitiveManager_new = CLIB.btGImpactCompoundShape_CompoundPrimitiveManager_new,
	GjkPairDetector_setCatchDegeneracies = CLIB.btGjkPairDetector_setCatchDegeneracies,
	SoftBody_Node_setX = CLIB.btSoftBody_Node_setX,
	SoftBody_appendNode = CLIB.btSoftBody_appendNode,
	CollisionAlgorithm_getAllContactManifolds = CLIB.btCollisionAlgorithm_getAllContactManifolds,
	Generic6DofSpring2Constraint_getCalculatedTransformA = CLIB.btGeneric6DofSpring2Constraint_getCalculatedTransformA,
	PairCachingGhostObject_new = CLIB.btPairCachingGhostObject_new,
	SoftBody_Note_getRank = CLIB.btSoftBody_Note_getRank,
	TypedConstraint_getRigidBodyA = CLIB.btTypedConstraint_getRigidBodyA,
	CollisionObject_internalGetExtensionPointer = CLIB.btCollisionObject_internalGetExtensionPointer,
	PrimitiveTriangle_applyTransform = CLIB.btPrimitiveTriangle_applyTransform,
	Chunk_setLength = CLIB.btChunk_setLength,
	DbvtAabbMm_Center = CLIB.btDbvtAabbMm_Center,
	RigidBody_computeGyroscopicForceExplicit = CLIB.btRigidBody_computeGyroscopicForceExplicit,
	Generic6DofSpringConstraint_setStiffness = CLIB.btGeneric6DofSpringConstraint_setStiffness,
	CollisionWorld_rayTestSingleInternal = CLIB.btCollisionWorld_rayTestSingleInternal,
	CollisionConfiguration_getPersistentManifoldPool = CLIB.btCollisionConfiguration_getPersistentManifoldPool,
	MultiBodyConstraint_getPosition = CLIB.btMultiBodyConstraint_getPosition,
	GImpactShapeInterface_getChildShape = CLIB.btGImpactShapeInterface_getChildShape,
	DefaultSerializer_new2 = CLIB.btDefaultSerializer_new2,
	MultiBody_localDirToWorld = CLIB.btMultiBody_localDirToWorld,
	BvhTree_getNodeData = CLIB.btBvhTree_getNodeData,
	GImpactShapeInterface_childrenHasTransform = CLIB.btGImpactShapeInterface_childrenHasTransform,
	ManifoldPoint_setContactERP = CLIB.btManifoldPoint_setContactERP,
	PrimitiveManagerBase_delete = CLIB.btPrimitiveManagerBase_delete,
	CollisionWorld_ConvexResultCallback_setClosestHitFraction = CLIB.btCollisionWorld_ConvexResultCallback_setClosestHitFraction,
	MultiBodyFixedConstraint_new2 = CLIB.btMultiBodyFixedConstraint_new2,
	Generic6DofConstraint_testAngularLimitMotor = CLIB.btGeneric6DofConstraint_testAngularLimitMotor,
	RotationalLimitMotor2_getSpringDampingLimited = CLIB.btRotationalLimitMotor2_getSpringDampingLimited,
	DbvtBroadphase_getUpdates_ratio = CLIB.btDbvtBroadphase_getUpdates_ratio,
	TransformUtil_calculateVelocity = CLIB.btTransformUtil_calculateVelocity,
	GImpactBvh_getNodeBound = CLIB.btGImpactBvh_getNodeBound,
	Generic6DofConstraint_new = CLIB.btGeneric6DofConstraint_new,
	CapsuleShape_new = CLIB.btCapsuleShape_new,
	GImpactMeshShapePart_TrimeshPrimitiveManager_getMargin = CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_getMargin,
	GImpactQuantizedBvh_getPrimitiveManager = CLIB.btGImpactQuantizedBvh_getPrimitiveManager,
	DefaultMotionState_setStartWorldTrans = CLIB.btDefaultMotionState_setStartWorldTrans,
	HingeAccumulatedAngleConstraint_new3 = CLIB.btHingeAccumulatedAngleConstraint_new3,
	Convex2dShape_getChildShape = CLIB.btConvex2dShape_getChildShape,
	CollisionObject_setCollisionFlags = CLIB.btCollisionObject_setCollisionFlags,
	SoftBody_RContact_new = CLIB.btSoftBody_RContact_new,
	DiscreteDynamicsWorld_updateVehicles = CLIB.btDiscreteDynamicsWorld_updateVehicles,
	SoftBody_checkContact = CLIB.btSoftBody_checkContact,
	TriangleMeshShape_localGetSupportingVertex = CLIB.btTriangleMeshShape_localGetSupportingVertex,
	GImpactCollisionAlgorithm_setPart0 = CLIB.btGImpactCollisionAlgorithm_setPart0,
	GjkPairDetector_setLastUsedMethod = CLIB.btGjkPairDetector_setLastUsedMethod,
	SoftBody_Link_delete = CLIB.btSoftBody_Link_delete,
	AABB_get_center_extend = CLIB.btAABB_get_center_extend,
	ContactSolverInfoData_setSingleAxisRollingFrictionThreshold = CLIB.btContactSolverInfoData_setSingleAxisRollingFrictionThreshold,
	TranslationalLimitMotor2_getServoTarget = CLIB.btTranslationalLimitMotor2_getServoTarget,
	QuantizedBvhTree_getEscapeNodeIndex = CLIB.btQuantizedBvhTree_getEscapeNodeIndex,
	RotationalLimitMotor2_setEquilibriumPoint = CLIB.btRotationalLimitMotor2_setEquilibriumPoint,
	AngularLimit_set = CLIB.btAngularLimit_set,
	QuantizedBvhTree_getNodeData = CLIB.btQuantizedBvhTree_getNodeData,
	TranslationalLimitMotor_setTargetVelocity = CLIB.btTranslationalLimitMotor_setTargetVelocity,
	QuantizedBvhTree_setNodeBound = CLIB.btQuantizedBvhTree_setNodeBound,
	GImpactQuantizedBvh_boxQuery = CLIB.btGImpactQuantizedBvh_boxQuery,
	SimulationIslandManager_updateActivationState = CLIB.btSimulationIslandManager_updateActivationState,
	RotationalLimitMotor2_setStopERP = CLIB.btRotationalLimitMotor2_setStopERP,
	ConvexCast_CastResult_setHitTransformA = CLIB.btConvexCast_CastResult_setHitTransformA,
	GjkPairDetector_setMinkowskiA = CLIB.btGjkPairDetector_setMinkowskiA,
	DbvtBroadphase_setCupdates = CLIB.btDbvtBroadphase_setCupdates,
	CollisionWorld_LocalShapeInfo_setShapePart = CLIB.btCollisionWorld_LocalShapeInfo_setShapePart,
	MultiBodyConstraintSolver_solveGroupCacheFriendlyFinish = CLIB.btMultiBodyConstraintSolver_solveGroupCacheFriendlyFinish,
	Generic6DofSpringConstraint_new = CLIB.btGeneric6DofSpringConstraint_new,
	ManifoldResult_calculateCombinedRollingFriction = CLIB.btManifoldResult_calculateCombinedRollingFriction,
	GImpactQuantizedBvh_update = CLIB.btGImpactQuantizedBvh_update,
	DynamicsWorld_removeConstraint = CLIB.btDynamicsWorld_removeConstraint,
	ConvexCast_calcTimeOfImpact = CLIB.btConvexCast_calcTimeOfImpact,
	Generic6DofSpring2Constraint_setTargetVelocity = CLIB.btGeneric6DofSpring2Constraint_setTargetVelocity,
	ConvexConcaveCollisionAlgorithm_SwappedCreateFunc_new = CLIB.btConvexConcaveCollisionAlgorithm_SwappedCreateFunc_new,
	Generic6DofSpring2Constraint_setRotationOrder = CLIB.btGeneric6DofSpring2Constraint_setRotationOrder,
	GImpactBvh_new = CLIB.btGImpactBvh_new,
	RotationalLimitMotor2_getStopCFM = CLIB.btRotationalLimitMotor2_getStopCFM,
	MultibodyLink_getJointDamping = CLIB.btMultibodyLink_getJointDamping,
	GImpactShapeInterface_processAllTrianglesRay = CLIB.btGImpactShapeInterface_processAllTrianglesRay,
	Dbvt_setFree = CLIB.btDbvt_setFree,
	SoftBody_new2 = CLIB.btSoftBody_new2,
	CylinderShapeZ_new = CLIB.btCylinderShapeZ_new,
	CollisionWorld_RayResultCallback_getFlags = CLIB.btCollisionWorld_RayResultCallback_getFlags,
	SoftBody_SContact_getMargin = CLIB.btSoftBody_SContact_getMargin,
	GImpactCompoundShape_CompoundPrimitiveManager_setCompoundShape = CLIB.btGImpactCompoundShape_CompoundPrimitiveManager_setCompoundShape,
	GImpactMeshShapePart_TrimeshPrimitiveManager_new = CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_new,
	AlignedObjectArray_btSoftBody_Anchor_size = CLIB.btAlignedObjectArray_btSoftBody_Anchor_size,
	GImpactMeshShapePart_TrimeshPrimitiveManager_getIndexstride = CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_getIndexstride,
	AxisSweep3_removeHandle = CLIB.btAxisSweep3_removeHandle,
	ConvexCast_CastResult_reportFailure = CLIB.btConvexCast_CastResult_reportFailure,
	SoftBody_appendTetra2 = CLIB.btSoftBody_appendTetra2,
	GImpactMeshShapePart_TrimeshPrimitiveManager_getNumverts = CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_getNumverts,
	MultiBody_addJointTorque = CLIB.btMultiBody_addJointTorque,
	Hinge2Constraint_getAnchor2 = CLIB.btHinge2Constraint_getAnchor2,
	GImpactMeshShapePart_TrimeshPrimitiveManager_setIndexbase = CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_setIndexbase,
	GImpactMeshShapePart_TrimeshPrimitiveManager_setMargin = CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_setMargin,
	GImpactMeshShapePart_TrimeshPrimitiveManager_get_vertex = CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_get_vertex,
	ConvexTriangleCallback_getTriangleCount = CLIB.btConvexTriangleCallback_getTriangleCount,
	GImpactMeshShapePart_TrimeshPrimitiveManager_setNumfaces = CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_setNumfaces,
	GImpactMeshShapePart_TrimeshPrimitiveManager_setScale = CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_setScale,
	DiscreteCollisionDetectorInterface_ClosestPointInput_setTransformA = CLIB.btDiscreteCollisionDetectorInterface_ClosestPointInput_setTransformA,
	CompoundShapeChild_getChildShapeType = CLIB.btCompoundShapeChild_getChildShapeType,
	DynamicsWorld_setGravity = CLIB.btDynamicsWorld_setGravity,
	GImpactMeshShapePart_getVertexCount = CLIB.btGImpactMeshShapePart_getVertexCount,
	Dbvt_countLeaves = CLIB.btDbvt_countLeaves,
	RotationalLimitMotor2_getCurrentPosition = CLIB.btRotationalLimitMotor2_getCurrentPosition,
	InternalTriangleIndexCallback_delete = CLIB.btInternalTriangleIndexCallback_delete,
	MultiBody_setJointVel = CLIB.btMultiBody_setJointVel,
	GjkPairDetector_getDegenerateSimplex = CLIB.btGjkPairDetector_getDegenerateSimplex,
	GImpactQuantizedBvh_getNodeData = CLIB.btGImpactQuantizedBvh_getNodeData,
	CollisionWorld_ClosestRayResultCallback_setHitNormalWorld = CLIB.btCollisionWorld_ClosestRayResultCallback_setHitNormalWorld,
	CollisionDispatcher_new = CLIB.btCollisionDispatcher_new,
	HingeConstraint_getFrameOffsetA = CLIB.btHingeConstraint_getFrameOffsetA,
	RigidBody_btRigidBodyConstructionInfo_getAdditionalLinearDampingThresholdSqr = CLIB.btRigidBody_btRigidBodyConstructionInfo_getAdditionalLinearDampingThresholdSqr,
	GjkPairDetector_setIgnoreMargin = CLIB.btGjkPairDetector_setIgnoreMargin,
	CollisionObject_getHitFraction = CLIB.btCollisionObject_getHitFraction,
	HeightfieldTerrainShape_new = CLIB.btHeightfieldTerrainShape_new,
	GImpactMeshShapePart_TrimeshPrimitiveManager_setIndicestype = CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_setIndicestype,
	DispatcherInfo_setAllowedCcdPenetration = CLIB.btDispatcherInfo_setAllowedCcdPenetration,
	MultibodyLink_getJointFeedback = CLIB.btMultibodyLink_getJointFeedback,
	DbvtBroadphase_getPaircache = CLIB.btDbvtBroadphase_getPaircache,
	GjkPairDetector_setMinkowskiB = CLIB.btGjkPairDetector_setMinkowskiB,
	ManifoldResult_getBody1Internal = CLIB.btManifoldResult_getBody1Internal,
	MultibodyLink_getMass = CLIB.btMultibodyLink_getMass,
	Dispatcher_allocateCollisionAlgorithm = CLIB.btDispatcher_allocateCollisionAlgorithm,
	Dbvt_sStkNPS_getMask = CLIB.btDbvt_sStkNPS_getMask,
	EmptyAlgorithm_CreateFunc_new = CLIB.btEmptyAlgorithm_CreateFunc_new,
	ConeTwistConstraint_getSolveTwistLimit = CLIB.btConeTwistConstraint_getSolveTwistLimit,
	OverlappingPairCache_getOverlappingPairArrayPtr = CLIB.btOverlappingPairCache_getOverlappingPairArrayPtr,
	MultiBody_getParentToLocalRot = CLIB.btMultiBody_getParentToLocalRot,
	CollisionWorld_AllHitsRayResultCallback_setRayFromWorld = CLIB.btCollisionWorld_AllHitsRayResultCallback_setRayFromWorld,
	HingeConstraint_getInfo1NonVirtual = CLIB.btHingeConstraint_getInfo1NonVirtual,
	HingeConstraint_getInfo2Internal = CLIB.btHingeConstraint_getInfo2Internal,
	DispatcherInfo_setStepCount = CLIB.btDispatcherInfo_setStepCount,
	TranslationalLimitMotor_needApplyForce = CLIB.btTranslationalLimitMotor_needApplyForce,
	HingeConstraint_getSolveLimit = CLIB.btHingeConstraint_getSolveLimit,
	HingeConstraint_getUpperLimit = CLIB.btHingeConstraint_getUpperLimit,
	IndexedMesh_getVertexType = CLIB.btIndexedMesh_getVertexType,
	ConeTwistConstraint_getTwistLimitSign = CLIB.btConeTwistConstraint_getTwistLimitSign,
	Generic6DofConstraint_setAngularUpperLimit = CLIB.btGeneric6DofConstraint_setAngularUpperLimit,
	SparseSdf_delete = CLIB.btSparseSdf_delete,
	SoftBody_Config_getAeromodel = CLIB.btSoftBody_Config_getAeromodel,
	JointFeedback_delete = CLIB.btJointFeedback_delete,
	DefaultMotionState_getStartWorldTrans = CLIB.btDefaultMotionState_getStartWorldTrans,
	RotationalLimitMotor_setCurrentLimitError = CLIB.btRotationalLimitMotor_setCurrentLimitError,
	SoftBody_Joint_getMassmatrix = CLIB.btSoftBody_Joint_getMassmatrix,
	MultiBodySolverConstraint_getRelpos1CrossNormal = CLIB.btMultiBodySolverConstraint_getRelpos1CrossNormal,
	CollisionObject_getInternalType = CLIB.btCollisionObject_getInternalType,
	Convex2dConvex2dAlgorithm_getManifold = CLIB.btConvex2dConvex2dAlgorithm_getManifold,
	Serializer_delete = CLIB.btSerializer_delete,
	AABB_overlapping_trans_conservative2 = CLIB.btAABB_overlapping_trans_conservative2,
	OptimizedBvhNode_getAabbMaxOrg = CLIB.btOptimizedBvhNode_getAabbMaxOrg,
	TranslationalLimitMotor2_setEquilibriumPoint = CLIB.btTranslationalLimitMotor2_setEquilibriumPoint,
	MultiBody_applyDeltaVeeMultiDof2 = CLIB.btMultiBody_applyDeltaVeeMultiDof2,
	CollisionWorld_RayResultCallback_getCollisionFilterMask = CLIB.btCollisionWorld_RayResultCallback_getCollisionFilterMask,
	AngularLimit_getLow = CLIB.btAngularLimit_getLow,
	RigidBody_getCenterOfMassTransform = CLIB.btRigidBody_getCenterOfMassTransform,
	VoronoiSimplexSolver_delete = CLIB.btVoronoiSimplexSolver_delete,
	CollisionDispatcher_getCollisionConfiguration = CLIB.btCollisionDispatcher_getCollisionConfiguration,
	Dispatcher_findAlgorithm = CLIB.btDispatcher_findAlgorithm,
	QuantizedBvhTree_quantizePoint = CLIB.btQuantizedBvhTree_quantizePoint,
	Generic6DofConstraint_setLimit = CLIB.btGeneric6DofConstraint_setLimit,
	AABB_appy_transform_trans_cache = CLIB.btAABB_appy_transform_trans_cache,
	SoftBody_getCdbvt = CLIB.btSoftBody_getCdbvt,
	CollisionWorld_ClosestConvexResultCallback_new = CLIB.btCollisionWorld_ClosestConvexResultCallback_new,
	GImpactShapeInterface_rayTest = CLIB.btGImpactShapeInterface_rayTest,
	ManifoldPoint_getAppliedImpulseLateral1 = CLIB.btManifoldPoint_getAppliedImpulseLateral1,
	QuantizedBvh_setQuantizationValues = CLIB.btQuantizedBvh_setQuantizationValues,
	DbvtBroadphase_getFixedleft = CLIB.btDbvtBroadphase_getFixedleft,
	ConvexTriangleCallback_setTimeStepAndCounters = CLIB.btConvexTriangleCallback_setTimeStepAndCounters,
	AABB_collide_plane = CLIB.btAABB_collide_plane,
	AlignedObjectArray_btSoftBody_Link_push_back = CLIB.btAlignedObjectArray_btSoftBody_Link_push_back,
	ManifoldPoint_getIndex0 = CLIB.btManifoldPoint_getIndex0,
	Generic6DofSpring2Constraint_setAxis = CLIB.btGeneric6DofSpring2Constraint_setAxis,
	CompoundFromGImpact_btCreateCompoundFromGimpactShape = CLIB.btCompoundFromGImpact_btCreateCompoundFromGimpactShape,
	Dbvt_getStkStack = CLIB.btDbvt_getStkStack,
	SoftBody_Cluster_getSelfCollisionImpulseFactor = CLIB.btSoftBody_Cluster_getSelfCollisionImpulseFactor,
	SoftBodyWorldInfo_new = CLIB.btSoftBodyWorldInfo_new,
	Generic6DofSpring2Constraint_btGetMatrixElem = CLIB.btGeneric6DofSpring2Constraint_btGetMatrixElem,
	MultiBody_clearVelocities = CLIB.btMultiBody_clearVelocities,
	DbvtNode_setData = CLIB.btDbvtNode_setData,
	RotationalLimitMotor2_isLimited = CLIB.btRotationalLimitMotor2_isLimited,
	ManifoldPoint_setDistance = CLIB.btManifoldPoint_setDistance,
	UnionFind_allocate = CLIB.btUnionFind_allocate,
	TranslationalLimitMotor2_isLimited = CLIB.btTranslationalLimitMotor2_isLimited,
	RotationalLimitMotor_setBounce = CLIB.btRotationalLimitMotor_setBounce,
	ManifoldPoint_setPositionWorldOnB = CLIB.btManifoldPoint_setPositionWorldOnB,
	SoftBody_Impulse_operator_n = CLIB.btSoftBody_Impulse_operator_n,
	TranslationalLimitMotor2_new = CLIB.btTranslationalLimitMotor2_new,
	AABB_collide_triangle_exact = CLIB.btAABB_collide_triangle_exact,
	ManifoldPoint_setAppliedImpulseLateral2 = CLIB.btManifoldPoint_setAppliedImpulseLateral2,
	ManifoldResult_getBody1Wrap = CLIB.btManifoldResult_getBody1Wrap,
	RotationalLimitMotor_getEnableMotor = CLIB.btRotationalLimitMotor_getEnableMotor,
	ManifoldResult_setBody0Wrap = CLIB.btManifoldResult_setBody0Wrap,
	RigidBody_btRigidBodyConstructionInfo_new2 = CLIB.btRigidBody_btRigidBodyConstructionInfo_new2,
	MinkowskiSumShape_getTransformA = CLIB.btMinkowskiSumShape_getTransformA,
	AxisSweep3_getNumHandles = CLIB.btAxisSweep3_getNumHandles,
	CollisionAlgorithmConstructionInfo_delete = CLIB.btCollisionAlgorithmConstructionInfo_delete,
	MultiBody_new = CLIB.btMultiBody_new,
	TranslationalLimitMotor_getLimitSoftness = CLIB.btTranslationalLimitMotor_getLimitSoftness,
	ConvexPolyhedron_project = CLIB.btConvexPolyhedron_project,
	DynamicsWorld_setInternalTickCallback = CLIB.btDynamicsWorld_setInternalTickCallback,
	AxisSweep3_new = CLIB.btAxisSweep3_new,
	MultiBody_addBaseTorque = CLIB.btMultiBody_addBaseTorque,
	MultiBody_goToSleep = CLIB.btMultiBody_goToSleep,
	MultiBody_addLinkForce = CLIB.btMultiBody_addLinkForce,
	AlignedObjectArray_btCollisionObjectPtr_size = CLIB.btAlignedObjectArray_btCollisionObjectPtr_size,
	MultiBody_getAngularMomentum = CLIB.btMultiBody_getAngularMomentum,
	Dbvt_sStkNP_getMask = CLIB.btDbvt_sStkNP_getMask,
	GImpactCollisionAlgorithm_getFace0 = CLIB.btGImpactCollisionAlgorithm_getFace0,
	MultiBody_getBaseTorque = CLIB.btMultiBody_getBaseTorque,
	MultiBody_getBaseWorldTransform = CLIB.btMultiBody_getBaseWorldTransform,
	MultiBody_getJointPos = CLIB.btMultiBody_getJointPos,
	IndexedMesh_setVertexBase = CLIB.btIndexedMesh_setVertexBase,
	MultiBody_getJointVelMultiDof = CLIB.btMultiBody_getJointVelMultiDof,
	MultiBody_getLinkMass = CLIB.btMultiBody_getLinkMass,
	MultiBodyJointMotor_setPositionTarget = CLIB.btMultiBodyJointMotor_setPositionTarget,
	Dbvt_ICollide_delete = CLIB.btDbvt_ICollide_delete,
	AlignedObjectArray_btCollisionObjectPtr_resizeNoInitialize = CLIB.btAlignedObjectArray_btCollisionObjectPtr_resizeNoInitialize,
	SphereShape_setUnscaledRadius = CLIB.btSphereShape_setUnscaledRadius,
	MultiBody_getNumLinks = CLIB.btMultiBody_getNumLinks,
	Point2PointConstraint_setPivotB = CLIB.btPoint2PointConstraint_setPivotB,
	Generic6DofConstraint_calculateTransforms = CLIB.btGeneric6DofConstraint_calculateTransforms,
	MultiBodyFixedConstraint_setPivotInB = CLIB.btMultiBodyFixedConstraint_setPivotInB,
	MultiBodyDynamicsWorld_addMultiBody = CLIB.btMultiBodyDynamicsWorld_addMultiBody,
	ConvexPolyhedron_getUniqueEdges = CLIB.btConvexPolyhedron_getUniqueEdges,
	MultiBodyDynamicsWorld_addMultiBodyConstraint = CLIB.btMultiBodyDynamicsWorld_addMultiBodyConstraint,
	MultiBodyFixedConstraint_getFrameInB = CLIB.btMultiBodyFixedConstraint_getFrameInB,
	RotationalLimitMotor2_setEnableSpring = CLIB.btRotationalLimitMotor2_setEnableSpring,
	CollisionWorld_LocalConvexResult_setHitCollisionObject = CLIB.btCollisionWorld_LocalConvexResult_setHitCollisionObject,
	HingeConstraint_testLimit = CLIB.btHingeConstraint_testLimit,
	TranslationalLimitMotor_getCurrentLinearDiff = CLIB.btTranslationalLimitMotor_getCurrentLinearDiff,
	ConvexHullShape_new = CLIB.btConvexHullShape_new,
}
library.e = {
}
library.metatables = {}
do -- btGImpactMeshShapePart_TrimeshPrimitiveManager_get_vertex
	local META = {}
	library.metatables.btGImpactMeshShapePart_TrimeshPrimitiveManager_get_vertex = META
	META.__index = function(s, k) return META[k] end
	function META:Count(...)
		return CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_get_vertex_count(self.ptr, ...)
	end
	function META:BtGImpactMeshShapePart_TrimeshPrimitiveManager_get_vertex(...)
		return CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_get_vertex(self.ptr, ...)
	end
end
do -- btGImpactMeshShapePart_TrimeshPrimitiveManager_get_bullet
	local META = {}
	library.metatables.btGImpactMeshShapePart_TrimeshPrimitiveManager_get_bullet = META
	META.__index = function(s, k) return META[k] end
	function META:Triangle(...)
		return CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_get_bullet_triangle(self.ptr, ...)
	end
end
do -- btGImpactMeshShapePart_TrimeshPrimitiveManager_setLock
	local META = {}
	library.metatables.btGImpactMeshShapePart_TrimeshPrimitiveManager_setLock = META
	META.__index = function(s, k) return META[k] end
	function META:Count(...)
		return CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_setLock_count(self.ptr, ...)
	end
end
do -- btGImpactMeshShapePart_TrimeshPrimitiveManager_getLock
	local META = {}
	library.metatables.btGImpactMeshShapePart_TrimeshPrimitiveManager_getLock = META
	META.__index = function(s, k) return META[k] end
	function META:Count(...)
		return CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_getLock_count(self.ptr, ...)
	end
end
do -- btCompoundCompoundCollisionAlgorithm_SwappedCreateFunc
	local META = {}
	library.metatables.btCompoundCompoundCollisionAlgorithm_SwappedCreateFunc = META
	META.__index = function(s, k) return META[k] end
	function library.CreateCompoundCompoundCollisionAlgorithm_SwappedCreateFunc(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btCompoundCompoundCollisionAlgorithm_SwappedCreateFunc_new(...)
		return self
	end
end
do -- btDiscreteCollisionDetectorInterface_ClosestPointInput
	local META = {}
	library.metatables.btDiscreteCollisionDetectorInterface_ClosestPointInput = META
	META.__index = function(s, k) return META[k] end
	function library.CreateDiscreteCollisionDetectorInterface_ClosestPointInput(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btDiscreteCollisionDetectorInterface_ClosestPointInput_new(...)
		return self
	end
	function META:GetTransformB(...)
		return CLIB.btDiscreteCollisionDetectorInterface_ClosestPointInput_getTransformB(self.ptr, ...)
	end
	function META:SetMaximumDistanceSquared(...)
		return CLIB.btDiscreteCollisionDetectorInterface_ClosestPointInput_setMaximumDistanceSquared(self.ptr, ...)
	end
	function META:GetTransformA(...)
		return CLIB.btDiscreteCollisionDetectorInterface_ClosestPointInput_getTransformA(self.ptr, ...)
	end
	function META:GetMaximumDistanceSquared(...)
		return CLIB.btDiscreteCollisionDetectorInterface_ClosestPointInput_getMaximumDistanceSquared(self.ptr, ...)
	end
	function META:Delete(...)
		return CLIB.btDiscreteCollisionDetectorInterface_ClosestPointInput_delete(self.ptr, ...)
	end
	function META:SetTransformB(...)
		return CLIB.btDiscreteCollisionDetectorInterface_ClosestPointInput_setTransformB(self.ptr, ...)
	end
	function META:SetTransformA(...)
		return CLIB.btDiscreteCollisionDetectorInterface_ClosestPointInput_setTransformA(self.ptr, ...)
	end
end
do -- btSoftBodyConcaveCollisionAlgorithm_SwappedCreateFunc
	local META = {}
	library.metatables.btSoftBodyConcaveCollisionAlgorithm_SwappedCreateFunc = META
	META.__index = function(s, k) return META[k] end
	function library.CreateSoftBodyConcaveCollisionAlgorithm_SwappedCreateFunc(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btSoftBodyConcaveCollisionAlgorithm_SwappedCreateFunc_new(...)
		return self
	end
end
do -- btConvexConcaveCollisionAlgorithm_SwappedCreateFunc
	local META = {}
	library.metatables.btConvexConcaveCollisionAlgorithm_SwappedCreateFunc = META
	META.__index = function(s, k) return META[k] end
	function library.CreateConvexConcaveCollisionAlgorithm_SwappedCreateFunc(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btConvexConcaveCollisionAlgorithm_SwappedCreateFunc_new(...)
		return self
	end
end
do -- btGImpactMeshShapePart_TrimeshPrimitiveManager_get
	local META = {}
	library.metatables.btGImpactMeshShapePart_TrimeshPrimitiveManager_get = META
	META.__index = function(s, k) return META[k] end
	function META:Ndexbase(...)
		return CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_getIndexbase(self.ptr, ...)
	end
	function META:Ndicestype(...)
		return CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_getIndicestype(self.ptr, ...)
	end
	function META:Ype(...)
		return CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_getType(self.ptr, ...)
	end
	function META:EshInterface(...)
		return CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_getMeshInterface(self.ptr, ...)
	end
	function META:Indices(...)
		return CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_get_indices(self.ptr, ...)
	end
	function META:Umfaces(...)
		return CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_getNumfaces(self.ptr, ...)
	end
	function META:Art(...)
		return CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_getPart(self.ptr, ...)
	end
	function META:Cale(...)
		return CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_getScale(self.ptr, ...)
	end
	function META:Tride(...)
		return CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_getStride(self.ptr, ...)
	end
	function META:Ertexbase(...)
		return CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_getVertexbase(self.ptr, ...)
	end
	function META:Argin(...)
		return CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_getMargin(self.ptr, ...)
	end
	function META:Ndexstride(...)
		return CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_getIndexstride(self.ptr, ...)
	end
	function META:Umverts(...)
		return CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_getNumverts(self.ptr, ...)
	end
end
do -- btAlignedObjectArray_btPersistentManifoldPtr_push
	local META = {}
	library.metatables.btAlignedObjectArray_btPersistentManifoldPtr_push = META
	META.__index = function(s, k) return META[k] end
	function META:Back(...)
		return CLIB.btAlignedObjectArray_btPersistentManifoldPtr_push_back(self.ptr, ...)
	end
end
do -- btPrimitiveTriangle_find_triangle_collision_clip
	local META = {}
	library.metatables.btPrimitiveTriangle_find_triangle_collision_clip = META
	META.__index = function(s, k) return META[k] end
	function META:Method(...)
		return CLIB.btPrimitiveTriangle_find_triangle_collision_clip_method(self.ptr, ...)
	end
end
do -- btAlignedObjectArray_btSoftBody_MaterialPtr_push
	local META = {}
	library.metatables.btAlignedObjectArray_btSoftBody_MaterialPtr_push = META
	META.__index = function(s, k) return META[k] end
	function META:Back(...)
		return CLIB.btAlignedObjectArray_btSoftBody_MaterialPtr_push_back(self.ptr, ...)
	end
end
do -- btAlignedObjectArray_btSoftBody_ClusterPtr_push
	local META = {}
	library.metatables.btAlignedObjectArray_btSoftBody_ClusterPtr_push = META
	META.__index = function(s, k) return META[k] end
	function META:Back(...)
		return CLIB.btAlignedObjectArray_btSoftBody_ClusterPtr_push_back(self.ptr, ...)
	end
end
do -- btGImpactCompoundShape_CompoundPrimitiveManager
	local META = {}
	library.metatables.btGImpactCompoundShape_CompoundPrimitiveManager = META
	META.__index = function(s, k) return META[k] end
	function library.CreateGImpactCompoundShape_CompoundPrimitiveManager(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btGImpactCompoundShape_CompoundPrimitiveManager_new3(...)
		return self
	end
	function library.CreateGImpactCompoundShape_CompoundPrimitiveManager2(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btGImpactCompoundShape_CompoundPrimitiveManager_new2(...)
		return self
	end
	function library.CreateGImpactCompoundShape_CompoundPrimitiveManager3(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btGImpactCompoundShape_CompoundPrimitiveManager_new(...)
		return self
	end
	function META:GetCompoundShape(...)
		return CLIB.btGImpactCompoundShape_CompoundPrimitiveManager_getCompoundShape(self.ptr, ...)
	end
	function META:SetCompoundShape(...)
		return CLIB.btGImpactCompoundShape_CompoundPrimitiveManager_setCompoundShape(self.ptr, ...)
	end
end
do -- btCompoundCompoundCollisionAlgorithm_CreateFunc
	local META = {}
	library.metatables.btCompoundCompoundCollisionAlgorithm_CreateFunc = META
	META.__index = function(s, k) return META[k] end
	function library.CreateCompoundCompoundCollisionAlgorithm_CreateFunc(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btCompoundCompoundCollisionAlgorithm_CreateFunc_new(...)
		return self
	end
end
do -- btSoftBodyConcaveCollisionAlgorithm_CreateFunc
	local META = {}
	library.metatables.btSoftBodyConcaveCollisionAlgorithm_CreateFunc = META
	META.__index = function(s, k) return META[k] end
	function library.CreateSoftBodyConcaveCollisionAlgorithm_CreateFunc(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btSoftBodyConcaveCollisionAlgorithm_CreateFunc_new(...)
		return self
	end
end
do -- btGImpactMeshShapePart_TrimeshPrimitiveManager
	local META = {}
	library.metatables.btGImpactMeshShapePart_TrimeshPrimitiveManager = META
	META.__index = function(s, k) return META[k] end
	function library.CreateGImpactMeshShapePart_TrimeshPrimitiveManager(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_new3(...)
		return self
	end
	function library.CreateGImpactMeshShapePart_TrimeshPrimitiveManager2(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_new2(...)
		return self
	end
	function library.CreateGImpactMeshShapePart_TrimeshPrimitiveManager3(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_new(...)
		return self
	end
	function META:SetPart(...)
		return CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_setPart(self.ptr, ...)
	end
	function META:SetIndexstride(...)
		return CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_setIndexstride(self.ptr, ...)
	end
	function META:SetMeshInterface(...)
		return CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_setMeshInterface(self.ptr, ...)
	end
	function META:Lock(...)
		return CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_lock(self.ptr, ...)
	end
	function META:SetNumverts(...)
		return CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_setNumverts(self.ptr, ...)
	end
	function META:SetVertexbase(...)
		return CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_setVertexbase(self.ptr, ...)
	end
	function META:SetStride(...)
		return CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_setStride(self.ptr, ...)
	end
	function META:Unlock(...)
		return CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_unlock(self.ptr, ...)
	end
	function META:SetType(...)
		return CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_setType(self.ptr, ...)
	end
	function META:SetIndexbase(...)
		return CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_setIndexbase(self.ptr, ...)
	end
	function META:SetMargin(...)
		return CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_setMargin(self.ptr, ...)
	end
	function META:SetNumfaces(...)
		return CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_setNumfaces(self.ptr, ...)
	end
	function META:SetScale(...)
		return CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_setScale(self.ptr, ...)
	end
	function META:SetIndicestype(...)
		return CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_setIndicestype(self.ptr, ...)
	end
end
do -- btCompoundCollisionAlgorithm_SwappedCreateFunc
	local META = {}
	library.metatables.btCompoundCollisionAlgorithm_SwappedCreateFunc = META
	META.__index = function(s, k) return META[k] end
	function library.CreateCompoundCollisionAlgorithm_SwappedCreateFunc(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btCompoundCollisionAlgorithm_SwappedCreateFunc_new(...)
		return self
	end
end
do -- btAlignedObjectArray_btCollisionObjectPtr_push
	local META = {}
	library.metatables.btAlignedObjectArray_btCollisionObjectPtr_push = META
	META.__index = function(s, k) return META[k] end
	function META:Back(...)
		return CLIB.btAlignedObjectArray_btCollisionObjectPtr_push_back(self.ptr, ...)
	end
end
do -- btAlignedObjectArray_btSoftBody_JointPtr_push
	local META = {}
	library.metatables.btAlignedObjectArray_btSoftBody_JointPtr_push = META
	META.__index = function(s, k) return META[k] end
	function META:Back(...)
		return CLIB.btAlignedObjectArray_btSoftBody_JointPtr_push_back(self.ptr, ...)
	end
end
do -- btCollisionWorld_ContactResultCallbackWrapper
	local META = {}
	library.metatables.btCollisionWorld_ContactResultCallbackWrapper = META
	META.__index = function(s, k) return META[k] end
	function library.CreateCollisionWorld_ContactResultCallbackWrapper(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btCollisionWorld_ContactResultCallbackWrapper_new(...)
		return self
	end
	function META:NeedsCollision(...)
		return CLIB.btCollisionWorld_ContactResultCallbackWrapper_needsCollision(self.ptr, ...)
	end
end
do -- btSphereTriangleCollisionAlgorithm_CreateFunc
	local META = {}
	library.metatables.btSphereTriangleCollisionAlgorithm_CreateFunc = META
	META.__index = function(s, k) return META[k] end
	function library.CreateSphereTriangleCollisionAlgorithm_CreateFunc(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btSphereTriangleCollisionAlgorithm_CreateFunc_new(...)
		return self
	end
end
do -- btRigidBody_computeGyroscopicImpulseImplicit
	local META = {}
	library.metatables.btRigidBody_computeGyroscopicImpulseImplicit = META
	META.__index = function(s, k) return META[k] end
	function META:World(...)
		return CLIB.btRigidBody_computeGyroscopicImpulseImplicit_World(self.ptr, ...)
	end
	function META:Body(...)
		return CLIB.btRigidBody_computeGyroscopicImpulseImplicit_Body(self.ptr, ...)
	end
end
do -- btConvexConcaveCollisionAlgorithm_CreateFunc
	local META = {}
	library.metatables.btConvexConcaveCollisionAlgorithm_CreateFunc = META
	META.__index = function(s, k) return META[k] end
	function library.CreateConvexConcaveCollisionAlgorithm_CreateFunc(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btConvexConcaveCollisionAlgorithm_CreateFunc_new(...)
		return self
	end
end
do -- btCollisionWorld_ClosestConvexResultCallback
	local META = {}
	library.metatables.btCollisionWorld_ClosestConvexResultCallback = META
	META.__index = function(s, k) return META[k] end
	function library.CreateCollisionWorld_ClosestConvexResultCallback(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btCollisionWorld_ClosestConvexResultCallback_new(...)
		return self
	end
	function META:SetHitCollisionObject(...)
		return CLIB.btCollisionWorld_ClosestConvexResultCallback_setHitCollisionObject(self.ptr, ...)
	end
	function META:GetConvexToWorld(...)
		return CLIB.btCollisionWorld_ClosestConvexResultCallback_getConvexToWorld(self.ptr, ...)
	end
	function META:GetConvexFromWorld(...)
		return CLIB.btCollisionWorld_ClosestConvexResultCallback_getConvexFromWorld(self.ptr, ...)
	end
	function META:GetHitNormalWorld(...)
		return CLIB.btCollisionWorld_ClosestConvexResultCallback_getHitNormalWorld(self.ptr, ...)
	end
	function META:GetHitPointWorld(...)
		return CLIB.btCollisionWorld_ClosestConvexResultCallback_getHitPointWorld(self.ptr, ...)
	end
	function META:SetConvexToWorld(...)
		return CLIB.btCollisionWorld_ClosestConvexResultCallback_setConvexToWorld(self.ptr, ...)
	end
	function META:GetHitCollisionObject(...)
		return CLIB.btCollisionWorld_ClosestConvexResultCallback_getHitCollisionObject(self.ptr, ...)
	end
	function META:SetConvexFromWorld(...)
		return CLIB.btCollisionWorld_ClosestConvexResultCallback_setConvexFromWorld(self.ptr, ...)
	end
	function META:SetHitNormalWorld(...)
		return CLIB.btCollisionWorld_ClosestConvexResultCallback_setHitNormalWorld(self.ptr, ...)
	end
	function META:SetHitPointWorld(...)
		return CLIB.btCollisionWorld_ClosestConvexResultCallback_setHitPointWorld(self.ptr, ...)
	end
end
do -- btAlignedObjectArray_btPersistentManifoldPtr
	local META = {}
	library.metatables.btAlignedObjectArray_btPersistentManifoldPtr = META
	META.__index = function(s, k) return META[k] end
	function library.CreateAlignedObjectArray_btPersistentManifoldPtr(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btAlignedObjectArray_btPersistentManifoldPtr_new(...)
		return self
	end
	function META:Size(...)
		return CLIB.btAlignedObjectArray_btPersistentManifoldPtr_size(self.ptr, ...)
	end
	function META:Delete(...)
		return CLIB.btAlignedObjectArray_btPersistentManifoldPtr_delete(self.ptr, ...)
	end
	function META:At(...)
		return CLIB.btAlignedObjectArray_btPersistentManifoldPtr_at(self.ptr, ...)
	end
	function META:ResizeNoInitialize(...)
		return CLIB.btAlignedObjectArray_btPersistentManifoldPtr_resizeNoInitialize(self.ptr, ...)
	end
end
do -- btCollisionWorld_ConvexResultCallbackWrapper
	local META = {}
	library.metatables.btCollisionWorld_ConvexResultCallbackWrapper = META
	META.__index = function(s, k) return META[k] end
	function library.CreateCollisionWorld_ConvexResultCallbackWrapper(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btCollisionWorld_ConvexResultCallbackWrapper_new(...)
		return self
	end
	function META:NeedsCollision(...)
		return CLIB.btCollisionWorld_ConvexResultCallbackWrapper_needsCollision(self.ptr, ...)
	end
end
do -- btAlignedObjectArray_btSoftBody_MaterialPtr
	local META = {}
	library.metatables.btAlignedObjectArray_btSoftBody_MaterialPtr = META
	META.__index = function(s, k) return META[k] end
	function META:Size(...)
		return CLIB.btAlignedObjectArray_btSoftBody_MaterialPtr_size(self.ptr, ...)
	end
	function META:ResizeNoInitialize(...)
		return CLIB.btAlignedObjectArray_btSoftBody_MaterialPtr_resizeNoInitialize(self.ptr, ...)
	end
	function META:At(...)
		return CLIB.btAlignedObjectArray_btSoftBody_MaterialPtr_at(self.ptr, ...)
	end
end
do -- btSphereSphereCollisionAlgorithm_CreateFunc
	local META = {}
	library.metatables.btSphereSphereCollisionAlgorithm_CreateFunc = META
	META.__index = function(s, k) return META[k] end
	function library.CreateSphereSphereCollisionAlgorithm_CreateFunc(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btSphereSphereCollisionAlgorithm_CreateFunc_new(...)
		return self
	end
end
do -- btDiscreteCollisionDetectorInterface_Result
	local META = {}
	library.metatables.btDiscreteCollisionDetectorInterface_Result = META
	META.__index = function(s, k) return META[k] end
	function META:SetShapeIdentifiersB(...)
		return CLIB.btDiscreteCollisionDetectorInterface_Result_setShapeIdentifiersB(self.ptr, ...)
	end
	function META:Delete(...)
		return CLIB.btDiscreteCollisionDetectorInterface_Result_delete(self.ptr, ...)
	end
	function META:AddContactPoint(...)
		return CLIB.btDiscreteCollisionDetectorInterface_Result_addContactPoint(self.ptr, ...)
	end
	function META:SetShapeIdentifiersA(...)
		return CLIB.btDiscreteCollisionDetectorInterface_Result_setShapeIdentifiersA(self.ptr, ...)
	end
end
do -- btAlignedObjectArray_btSoftBody_Anchor_push
	local META = {}
	library.metatables.btAlignedObjectArray_btSoftBody_Anchor_push = META
	META.__index = function(s, k) return META[k] end
	function META:Back(...)
		return CLIB.btAlignedObjectArray_btSoftBody_Anchor_push_back(self.ptr, ...)
	end
end
do -- btVehicleRaycaster_btVehicleRaycasterResult
	local META = {}
	library.metatables.btVehicleRaycaster_btVehicleRaycasterResult = META
	META.__index = function(s, k) return META[k] end
	function library.CreateVehicleRaycaster_btVehicleRaycasterResult(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btVehicleRaycaster_btVehicleRaycasterResult_new(...)
		return self
	end
	function META:SetHitPointInWorld(...)
		return CLIB.btVehicleRaycaster_btVehicleRaycasterResult_setHitPointInWorld(self.ptr, ...)
	end
	function META:SetHitNormalInWorld(...)
		return CLIB.btVehicleRaycaster_btVehicleRaycasterResult_setHitNormalInWorld(self.ptr, ...)
	end
	function META:SetDistFraction(...)
		return CLIB.btVehicleRaycaster_btVehicleRaycasterResult_setDistFraction(self.ptr, ...)
	end
	function META:GetHitPointInWorld(...)
		return CLIB.btVehicleRaycaster_btVehicleRaycasterResult_getHitPointInWorld(self.ptr, ...)
	end
	function META:GetDistFraction(...)
		return CLIB.btVehicleRaycaster_btVehicleRaycasterResult_getDistFraction(self.ptr, ...)
	end
	function META:Delete(...)
		return CLIB.btVehicleRaycaster_btVehicleRaycasterResult_delete(self.ptr, ...)
	end
	function META:GetHitNormalInWorld(...)
		return CLIB.btVehicleRaycaster_btVehicleRaycasterResult_getHitNormalInWorld(self.ptr, ...)
	end
end
do -- btAlignedObjectArray_btSoftBody_Node_index
	local META = {}
	library.metatables.btAlignedObjectArray_btSoftBody_Node_index = META
	META.__index = function(s, k) return META[k] end
	function META:Of(...)
		return CLIB.btAlignedObjectArray_btSoftBody_Node_index_of(self.ptr, ...)
	end
end
do -- btAlignedObjectArray_btBroadphasePair_push
	local META = {}
	library.metatables.btAlignedObjectArray_btBroadphasePair_push = META
	META.__index = function(s, k) return META[k] end
	function META:Back(...)
		return CLIB.btAlignedObjectArray_btBroadphasePair_push_back(self.ptr, ...)
	end
end
do -- btAlignedObjectArray_btSoftBody_Tetra_push
	local META = {}
	library.metatables.btAlignedObjectArray_btSoftBody_Tetra_push = META
	META.__index = function(s, k) return META[k] end
	function META:Back(...)
		return CLIB.btAlignedObjectArray_btSoftBody_Tetra_push_back(self.ptr, ...)
	end
end
do -- btConvexPlaneCollisionAlgorithm_CreateFunc
	local META = {}
	library.metatables.btConvexPlaneCollisionAlgorithm_CreateFunc = META
	META.__index = function(s, k) return META[k] end
	function library.CreateConvexPlaneCollisionAlgorithm_CreateFunc(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btConvexPlaneCollisionAlgorithm_CreateFunc_new(...)
		return self
	end
	function META:SetNumPerturbationIterations(...)
		return CLIB.btConvexPlaneCollisionAlgorithm_CreateFunc_setNumPerturbationIterations(self.ptr, ...)
	end
	function META:SetMinimumPointsPerturbationThreshold(...)
		return CLIB.btConvexPlaneCollisionAlgorithm_CreateFunc_setMinimumPointsPerturbationThreshold(self.ptr, ...)
	end
	function META:GetNumPerturbationIterations(...)
		return CLIB.btConvexPlaneCollisionAlgorithm_CreateFunc_getNumPerturbationIterations(self.ptr, ...)
	end
	function META:GetMinimumPointsPerturbationThreshold(...)
		return CLIB.btConvexPlaneCollisionAlgorithm_CreateFunc_getMinimumPointsPerturbationThreshold(self.ptr, ...)
	end
end
do -- btAlignedObjectArray_btSoftBody_ClusterPtr
	local META = {}
	library.metatables.btAlignedObjectArray_btSoftBody_ClusterPtr = META
	META.__index = function(s, k) return META[k] end
	function META:Size(...)
		return CLIB.btAlignedObjectArray_btSoftBody_ClusterPtr_size(self.ptr, ...)
	end
	function META:ResizeNoInitialize(...)
		return CLIB.btAlignedObjectArray_btSoftBody_ClusterPtr_resizeNoInitialize(self.ptr, ...)
	end
	function META:At(...)
		return CLIB.btAlignedObjectArray_btSoftBody_ClusterPtr_at(self.ptr, ...)
	end
end
do -- btCollisionWorld_ClosestRayResultCallback
	local META = {}
	library.metatables.btCollisionWorld_ClosestRayResultCallback = META
	META.__index = function(s, k) return META[k] end
	function library.CreateCollisionWorld_ClosestRayResultCallback(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btCollisionWorld_ClosestRayResultCallback_new(...)
		return self
	end
	function META:SetRayFromWorld(...)
		return CLIB.btCollisionWorld_ClosestRayResultCallback_setRayFromWorld(self.ptr, ...)
	end
	function META:SetRayToWorld(...)
		return CLIB.btCollisionWorld_ClosestRayResultCallback_setRayToWorld(self.ptr, ...)
	end
	function META:GetRayFromWorld(...)
		return CLIB.btCollisionWorld_ClosestRayResultCallback_getRayFromWorld(self.ptr, ...)
	end
	function META:GetHitPointWorld(...)
		return CLIB.btCollisionWorld_ClosestRayResultCallback_getHitPointWorld(self.ptr, ...)
	end
	function META:GetHitNormalWorld(...)
		return CLIB.btCollisionWorld_ClosestRayResultCallback_getHitNormalWorld(self.ptr, ...)
	end
	function META:SetHitPointWorld(...)
		return CLIB.btCollisionWorld_ClosestRayResultCallback_setHitPointWorld(self.ptr, ...)
	end
	function META:GetRayToWorld(...)
		return CLIB.btCollisionWorld_ClosestRayResultCallback_getRayToWorld(self.ptr, ...)
	end
	function META:SetHitNormalWorld(...)
		return CLIB.btCollisionWorld_ClosestRayResultCallback_setHitNormalWorld(self.ptr, ...)
	end
end
do -- btAlignedObjectArray_btSoftBody_Node_push
	local META = {}
	library.metatables.btAlignedObjectArray_btSoftBody_Node_push = META
	META.__index = function(s, k) return META[k] end
	function META:Back(...)
		return CLIB.btAlignedObjectArray_btSoftBody_Node_push_back(self.ptr, ...)
	end
end
do -- btSoftBodyRigidBodyCollisionConfiguration
	local META = {}
	library.metatables.btSoftBodyRigidBodyCollisionConfiguration = META
	META.__index = function(s, k) return META[k] end
	function library.CreateSoftBodyRigidBodyCollisionConfiguration(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btSoftBodyRigidBodyCollisionConfiguration_new(...)
		return self
	end
	function library.CreateSoftBodyRigidBodyCollisionConfiguration2(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btSoftBodyRigidBodyCollisionConfiguration_new2(...)
		return self
	end
end
do -- btCollisionWorld_RayResultCallbackWrapper
	local META = {}
	library.metatables.btCollisionWorld_RayResultCallbackWrapper = META
	META.__index = function(s, k) return META[k] end
	function library.CreateCollisionWorld_RayResultCallbackWrapper(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btCollisionWorld_RayResultCallbackWrapper_new(...)
		return self
	end
	function META:NeedsCollision(...)
		return CLIB.btCollisionWorld_RayResultCallbackWrapper_needsCollision(self.ptr, ...)
	end
end
do -- btAlignedObjectArray_btCollisionObjectPtr
	local META = {}
	library.metatables.btAlignedObjectArray_btCollisionObjectPtr = META
	META.__index = function(s, k) return META[k] end
	function META:At(...)
		return CLIB.btAlignedObjectArray_btCollisionObjectPtr_at(self.ptr, ...)
	end
	function META:FindLinearSearch2(...)
		return CLIB.btAlignedObjectArray_btCollisionObjectPtr_findLinearSearch2(self.ptr, ...)
	end
	function META:Size(...)
		return CLIB.btAlignedObjectArray_btCollisionObjectPtr_size(self.ptr, ...)
	end
	function META:ResizeNoInitialize(...)
		return CLIB.btAlignedObjectArray_btCollisionObjectPtr_resizeNoInitialize(self.ptr, ...)
	end
end
do -- btAlignedObjectArray_btSoftBody_Face_push
	local META = {}
	library.metatables.btAlignedObjectArray_btSoftBody_Face_push = META
	META.__index = function(s, k) return META[k] end
	function META:Back(...)
		return CLIB.btAlignedObjectArray_btSoftBody_Face_push_back(self.ptr, ...)
	end
end
do -- btCollisionWorld_AllHitsRayResultCallback
	local META = {}
	library.metatables.btCollisionWorld_AllHitsRayResultCallback = META
	META.__index = function(s, k) return META[k] end
	function library.CreateCollisionWorld_AllHitsRayResultCallback(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btCollisionWorld_AllHitsRayResultCallback_new(...)
		return self
	end
	function META:GetCollisionObjects(...)
		return CLIB.btCollisionWorld_AllHitsRayResultCallback_getCollisionObjects(self.ptr, ...)
	end
	function META:GetHitFractions(...)
		return CLIB.btCollisionWorld_AllHitsRayResultCallback_getHitFractions(self.ptr, ...)
	end
	function META:GetRayToWorld(...)
		return CLIB.btCollisionWorld_AllHitsRayResultCallback_getRayToWorld(self.ptr, ...)
	end
	function META:GetHitNormalWorld(...)
		return CLIB.btCollisionWorld_AllHitsRayResultCallback_getHitNormalWorld(self.ptr, ...)
	end
	function META:GetHitPointWorld(...)
		return CLIB.btCollisionWorld_AllHitsRayResultCallback_getHitPointWorld(self.ptr, ...)
	end
	function META:GetRayFromWorld(...)
		return CLIB.btCollisionWorld_AllHitsRayResultCallback_getRayFromWorld(self.ptr, ...)
	end
	function META:SetRayToWorld(...)
		return CLIB.btCollisionWorld_AllHitsRayResultCallback_setRayToWorld(self.ptr, ...)
	end
	function META:SetRayFromWorld(...)
		return CLIB.btCollisionWorld_AllHitsRayResultCallback_setRayFromWorld(self.ptr, ...)
	end
end
do -- btBox2dBox2dCollisionAlgorithm_CreateFunc
	local META = {}
	library.metatables.btBox2dBox2dCollisionAlgorithm_CreateFunc = META
	META.__index = function(s, k) return META[k] end
	function library.CreateBox2dBox2dCollisionAlgorithm_CreateFunc(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btBox2dBox2dCollisionAlgorithm_CreateFunc_new(...)
		return self
	end
end
do -- btAlignedObjectArray_btSoftBody_Link_push
	local META = {}
	library.metatables.btAlignedObjectArray_btSoftBody_Link_push = META
	META.__index = function(s, k) return META[k] end
	function META:Back(...)
		return CLIB.btAlignedObjectArray_btSoftBody_Link_push_back(self.ptr, ...)
	end
end
do -- btSoftRigidCollisionAlgorithm_CreateFunc
	local META = {}
	library.metatables.btSoftRigidCollisionAlgorithm_CreateFunc = META
	META.__index = function(s, k) return META[k] end
	function library.CreateSoftRigidCollisionAlgorithm_CreateFunc(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btSoftRigidCollisionAlgorithm_CreateFunc_new(...)
		return self
	end
end
do -- btSimulationIslandManager_IslandCallback
	local META = {}
	library.metatables.btSimulationIslandManager_IslandCallback = META
	META.__index = function(s, k) return META[k] end
	function META:Delete(...)
		return CLIB.btSimulationIslandManager_IslandCallback_delete(self.ptr, ...)
	end
	function META:ProcessIsland(...)
		return CLIB.btSimulationIslandManager_IslandCallback_processIsland(self.ptr, ...)
	end
end
do -- btSphereBoxCollisionAlgorithm_CreateFunc
	local META = {}
	library.metatables.btSphereBoxCollisionAlgorithm_CreateFunc = META
	META.__index = function(s, k) return META[k] end
	function library.CreateSphereBoxCollisionAlgorithm_CreateFunc(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btSphereBoxCollisionAlgorithm_CreateFunc_new(...)
		return self
	end
end
do -- btAlignedObjectArray_btSoftBody_JointPtr
	local META = {}
	library.metatables.btAlignedObjectArray_btSoftBody_JointPtr = META
	META.__index = function(s, k) return META[k] end
	function META:ResizeNoInitialize(...)
		return CLIB.btAlignedObjectArray_btSoftBody_JointPtr_resizeNoInitialize(self.ptr, ...)
	end
	function META:Size(...)
		return CLIB.btAlignedObjectArray_btSoftBody_JointPtr_size(self.ptr, ...)
	end
	function META:At(...)
		return CLIB.btAlignedObjectArray_btSoftBody_JointPtr_at(self.ptr, ...)
	end
end
do -- btRigidBody_btRigidBodyConstructionInfo
	local META = {}
	library.metatables.btRigidBody_btRigidBodyConstructionInfo = META
	META.__index = function(s, k) return META[k] end
	function library.CreateRigidBody_btRigidBodyConstructionInfo(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btRigidBody_btRigidBodyConstructionInfo_new(...)
		return self
	end
	function library.CreateRigidBody_btRigidBodyConstructionInfo2(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btRigidBody_btRigidBodyConstructionInfo_new2(...)
		return self
	end
	function META:SetAdditionalLinearDampingThresholdSqr(...)
		return CLIB.btRigidBody_btRigidBodyConstructionInfo_setAdditionalLinearDampingThresholdSqr(self.ptr, ...)
	end
	function META:GetAdditionalAngularDampingThresholdSqr(...)
		return CLIB.btRigidBody_btRigidBodyConstructionInfo_getAdditionalAngularDampingThresholdSqr(self.ptr, ...)
	end
	function META:GetAdditionalAngularDampingFactor(...)
		return CLIB.btRigidBody_btRigidBodyConstructionInfo_getAdditionalAngularDampingFactor(self.ptr, ...)
	end
	function META:Delete(...)
		return CLIB.btRigidBody_btRigidBodyConstructionInfo_delete(self.ptr, ...)
	end
	function META:SetLocalInertia(...)
		return CLIB.btRigidBody_btRigidBodyConstructionInfo_setLocalInertia(self.ptr, ...)
	end
	function META:SetMotionState(...)
		return CLIB.btRigidBody_btRigidBodyConstructionInfo_setMotionState(self.ptr, ...)
	end
	function META:GetAdditionalDamping(...)
		return CLIB.btRigidBody_btRigidBodyConstructionInfo_getAdditionalDamping(self.ptr, ...)
	end
	function META:GetAdditionalDampingFactor(...)
		return CLIB.btRigidBody_btRigidBodyConstructionInfo_getAdditionalDampingFactor(self.ptr, ...)
	end
	function META:GetLocalInertia(...)
		return CLIB.btRigidBody_btRigidBodyConstructionInfo_getLocalInertia(self.ptr, ...)
	end
	function META:SetCollisionShape(...)
		return CLIB.btRigidBody_btRigidBodyConstructionInfo_setCollisionShape(self.ptr, ...)
	end
	function META:SetStartWorldTransform(...)
		return CLIB.btRigidBody_btRigidBodyConstructionInfo_setStartWorldTransform(self.ptr, ...)
	end
	function META:SetAdditionalAngularDampingThresholdSqr(...)
		return CLIB.btRigidBody_btRigidBodyConstructionInfo_setAdditionalAngularDampingThresholdSqr(self.ptr, ...)
	end
	function META:GetFriction(...)
		return CLIB.btRigidBody_btRigidBodyConstructionInfo_getFriction(self.ptr, ...)
	end
	function META:SetLinearSleepingThreshold(...)
		return CLIB.btRigidBody_btRigidBodyConstructionInfo_setLinearSleepingThreshold(self.ptr, ...)
	end
	function META:GetRollingFriction(...)
		return CLIB.btRigidBody_btRigidBodyConstructionInfo_getRollingFriction(self.ptr, ...)
	end
	function META:SetRollingFriction(...)
		return CLIB.btRigidBody_btRigidBodyConstructionInfo_setRollingFriction(self.ptr, ...)
	end
	function META:SetRestitution(...)
		return CLIB.btRigidBody_btRigidBodyConstructionInfo_setRestitution(self.ptr, ...)
	end
	function META:SetMass(...)
		return CLIB.btRigidBody_btRigidBodyConstructionInfo_setMass(self.ptr, ...)
	end
	function META:SetLinearDamping(...)
		return CLIB.btRigidBody_btRigidBodyConstructionInfo_setLinearDamping(self.ptr, ...)
	end
	function META:SetFriction(...)
		return CLIB.btRigidBody_btRigidBodyConstructionInfo_setFriction(self.ptr, ...)
	end
	function META:SetAngularSleepingThreshold(...)
		return CLIB.btRigidBody_btRigidBodyConstructionInfo_setAngularSleepingThreshold(self.ptr, ...)
	end
	function META:SetAngularDamping(...)
		return CLIB.btRigidBody_btRigidBodyConstructionInfo_setAngularDamping(self.ptr, ...)
	end
	function META:SetAdditionalDampingFactor(...)
		return CLIB.btRigidBody_btRigidBodyConstructionInfo_setAdditionalDampingFactor(self.ptr, ...)
	end
	function META:SetAdditionalDamping(...)
		return CLIB.btRigidBody_btRigidBodyConstructionInfo_setAdditionalDamping(self.ptr, ...)
	end
	function META:SetAdditionalAngularDampingFactor(...)
		return CLIB.btRigidBody_btRigidBodyConstructionInfo_setAdditionalAngularDampingFactor(self.ptr, ...)
	end
	function META:GetStartWorldTransform(...)
		return CLIB.btRigidBody_btRigidBodyConstructionInfo_getStartWorldTransform(self.ptr, ...)
	end
	function META:GetRestitution(...)
		return CLIB.btRigidBody_btRigidBodyConstructionInfo_getRestitution(self.ptr, ...)
	end
	function META:GetMass(...)
		return CLIB.btRigidBody_btRigidBodyConstructionInfo_getMass(self.ptr, ...)
	end
	function META:GetLinearDamping(...)
		return CLIB.btRigidBody_btRigidBodyConstructionInfo_getLinearDamping(self.ptr, ...)
	end
	function META:GetCollisionShape(...)
		return CLIB.btRigidBody_btRigidBodyConstructionInfo_getCollisionShape(self.ptr, ...)
	end
	function META:GetAngularSleepingThreshold(...)
		return CLIB.btRigidBody_btRigidBodyConstructionInfo_getAngularSleepingThreshold(self.ptr, ...)
	end
	function META:GetAngularDamping(...)
		return CLIB.btRigidBody_btRigidBodyConstructionInfo_getAngularDamping(self.ptr, ...)
	end
	function META:GetLinearSleepingThreshold(...)
		return CLIB.btRigidBody_btRigidBodyConstructionInfo_getLinearSleepingThreshold(self.ptr, ...)
	end
	function META:GetMotionState(...)
		return CLIB.btRigidBody_btRigidBodyConstructionInfo_getMotionState(self.ptr, ...)
	end
	function META:GetAdditionalLinearDampingThresholdSqr(...)
		return CLIB.btRigidBody_btRigidBodyConstructionInfo_getAdditionalLinearDampingThresholdSqr(self.ptr, ...)
	end
end
do -- btCompoundCollisionAlgorithm_CreateFunc
	local META = {}
	library.metatables.btCompoundCollisionAlgorithm_CreateFunc = META
	META.__index = function(s, k) return META[k] end
	function library.CreateCompoundCollisionAlgorithm_CreateFunc(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btCompoundCollisionAlgorithm_CreateFunc_new(...)
		return self
	end
end
do -- btAlignedObjectArray_btIndexedMesh_push
	local META = {}
	library.metatables.btAlignedObjectArray_btIndexedMesh_push = META
	META.__index = function(s, k) return META[k] end
	function META:Back(...)
		return CLIB.btAlignedObjectArray_btIndexedMesh_push_back(self.ptr, ...)
	end
end
do -- btGeneric6DofConstraint_get_limit_motor
	local META = {}
	library.metatables.btGeneric6DofConstraint_get_limit_motor = META
	META.__index = function(s, k) return META[k] end
	function META:Info2(...)
		return CLIB.btGeneric6DofConstraint_get_limit_motor_info2(self.ptr, ...)
	end
end
do -- btSoftSoftCollisionAlgorithm_CreateFunc
	local META = {}
	library.metatables.btSoftSoftCollisionAlgorithm_CreateFunc = META
	META.__index = function(s, k) return META[k] end
	function library.CreateSoftSoftCollisionAlgorithm_CreateFunc(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btSoftSoftCollisionAlgorithm_CreateFunc_new(...)
		return self
	end
end
do -- btAlignedObjectArray_btSoftBodyPtr_push
	local META = {}
	library.metatables.btAlignedObjectArray_btSoftBodyPtr_push = META
	META.__index = function(s, k) return META[k] end
	function META:Back(...)
		return CLIB.btAlignedObjectArray_btSoftBodyPtr_push_back(self.ptr, ...)
	end
end
do -- btAlignedObjectArray_btSoftBody_Anchor
	local META = {}
	library.metatables.btAlignedObjectArray_btSoftBody_Anchor = META
	META.__index = function(s, k) return META[k] end
	function META:ResizeNoInitialize(...)
		return CLIB.btAlignedObjectArray_btSoftBody_Anchor_resizeNoInitialize(self.ptr, ...)
	end
	function META:At(...)
		return CLIB.btAlignedObjectArray_btSoftBody_Anchor_at(self.ptr, ...)
	end
	function META:Size(...)
		return CLIB.btAlignedObjectArray_btSoftBody_Anchor_size(self.ptr, ...)
	end
end
do -- btCollisionWorld_ContactResultCallback
	local META = {}
	library.metatables.btCollisionWorld_ContactResultCallback = META
	META.__index = function(s, k) return META[k] end
	function META:SetClosestDistanceThreshold(...)
		return CLIB.btCollisionWorld_ContactResultCallback_setClosestDistanceThreshold(self.ptr, ...)
	end
	function META:SetCollisionFilterGroup(...)
		return CLIB.btCollisionWorld_ContactResultCallback_setCollisionFilterGroup(self.ptr, ...)
	end
	function META:GetCollisionFilterGroup(...)
		return CLIB.btCollisionWorld_ContactResultCallback_getCollisionFilterGroup(self.ptr, ...)
	end
	function META:AddSingleResult(...)
		return CLIB.btCollisionWorld_ContactResultCallback_addSingleResult(self.ptr, ...)
	end
	function META:NeedsCollision(...)
		return CLIB.btCollisionWorld_ContactResultCallback_needsCollision(self.ptr, ...)
	end
	function META:GetCollisionFilterMask(...)
		return CLIB.btCollisionWorld_ContactResultCallback_getCollisionFilterMask(self.ptr, ...)
	end
	function META:GetClosestDistanceThreshold(...)
		return CLIB.btCollisionWorld_ContactResultCallback_getClosestDistanceThreshold(self.ptr, ...)
	end
	function META:SetCollisionFilterMask(...)
		return CLIB.btCollisionWorld_ContactResultCallback_setCollisionFilterMask(self.ptr, ...)
	end
	function META:Delete(...)
		return CLIB.btCollisionWorld_ContactResultCallback_delete(self.ptr, ...)
	end
end
do -- btGImpactCollisionAlgorithm_gimpact_vs
	local META = {}
	library.metatables.btGImpactCollisionAlgorithm_gimpact_vs = META
	META.__index = function(s, k) return META[k] end
	function META:Concave(...)
		return CLIB.btGImpactCollisionAlgorithm_gimpact_vs_concave(self.ptr, ...)
	end
	function META:Shape(...)
		return CLIB.btGImpactCollisionAlgorithm_gimpact_vs_shape(self.ptr, ...)
	end
	function META:Gimpact(...)
		return CLIB.btGImpactCollisionAlgorithm_gimpact_vs_gimpact(self.ptr, ...)
	end
	function META:Compoundshape(...)
		return CLIB.btGImpactCollisionAlgorithm_gimpact_vs_compoundshape(self.ptr, ...)
	end
end
do -- btInternalTriangleIndexCallbackWrapper
	local META = {}
	library.metatables.btInternalTriangleIndexCallbackWrapper = META
	META.__index = function(s, k) return META[k] end
	function library.CreateInternalTriangleIndexCallbackWrapper(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btInternalTriangleIndexCallbackWrapper_new(...)
		return self
	end
end
do -- btConvex2dConvex2dAlgorithm_CreateFunc
	local META = {}
	library.metatables.btConvex2dConvex2dAlgorithm_CreateFunc = META
	META.__index = function(s, k) return META[k] end
	function library.CreateConvex2dConvex2dAlgorithm_CreateFunc(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btConvex2dConvex2dAlgorithm_CreateFunc_new(...)
		return self
	end
	function META:GetMinimumPointsPerturbationThreshold(...)
		return CLIB.btConvex2dConvex2dAlgorithm_CreateFunc_getMinimumPointsPerturbationThreshold(self.ptr, ...)
	end
	function META:SetPdSolver(...)
		return CLIB.btConvex2dConvex2dAlgorithm_CreateFunc_setPdSolver(self.ptr, ...)
	end
	function META:SetSimplexSolver(...)
		return CLIB.btConvex2dConvex2dAlgorithm_CreateFunc_setSimplexSolver(self.ptr, ...)
	end
	function META:SetMinimumPointsPerturbationThreshold(...)
		return CLIB.btConvex2dConvex2dAlgorithm_CreateFunc_setMinimumPointsPerturbationThreshold(self.ptr, ...)
	end
	function META:SetNumPerturbationIterations(...)
		return CLIB.btConvex2dConvex2dAlgorithm_CreateFunc_setNumPerturbationIterations(self.ptr, ...)
	end
	function META:GetSimplexSolver(...)
		return CLIB.btConvex2dConvex2dAlgorithm_CreateFunc_getSimplexSolver(self.ptr, ...)
	end
	function META:GetNumPerturbationIterations(...)
		return CLIB.btConvex2dConvex2dAlgorithm_CreateFunc_getNumPerturbationIterations(self.ptr, ...)
	end
	function META:GetPdSolver(...)
		return CLIB.btConvex2dConvex2dAlgorithm_CreateFunc_getPdSolver(self.ptr, ...)
	end
end
do -- btGImpactCollisionAlgorithm_CreateFunc
	local META = {}
	library.metatables.btGImpactCollisionAlgorithm_CreateFunc = META
	META.__index = function(s, k) return META[k] end
	function library.CreateGImpactCollisionAlgorithm_CreateFunc(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btGImpactCollisionAlgorithm_CreateFunc_new(...)
		return self
	end
end
do -- btAlignedObjectArray_btBroadphasePair
	local META = {}
	library.metatables.btAlignedObjectArray_btBroadphasePair = META
	META.__index = function(s, k) return META[k] end
	function META:ResizeNoInitialize(...)
		return CLIB.btAlignedObjectArray_btBroadphasePair_resizeNoInitialize(self.ptr, ...)
	end
	function META:At(...)
		return CLIB.btAlignedObjectArray_btBroadphasePair_at(self.ptr, ...)
	end
	function META:Size(...)
		return CLIB.btAlignedObjectArray_btBroadphasePair_size(self.ptr, ...)
	end
end
do -- btCollisionWorld_ConvexResultCallback
	local META = {}
	library.metatables.btCollisionWorld_ConvexResultCallback = META
	META.__index = function(s, k) return META[k] end
	function META:SetCollisionFilterMask(...)
		return CLIB.btCollisionWorld_ConvexResultCallback_setCollisionFilterMask(self.ptr, ...)
	end
	function META:NeedsCollision(...)
		return CLIB.btCollisionWorld_ConvexResultCallback_needsCollision(self.ptr, ...)
	end
	function META:GetClosestHitFraction(...)
		return CLIB.btCollisionWorld_ConvexResultCallback_getClosestHitFraction(self.ptr, ...)
	end
	function META:AddSingleResult(...)
		return CLIB.btCollisionWorld_ConvexResultCallback_addSingleResult(self.ptr, ...)
	end
	function META:GetCollisionFilterGroup(...)
		return CLIB.btCollisionWorld_ConvexResultCallback_getCollisionFilterGroup(self.ptr, ...)
	end
	function META:Delete(...)
		return CLIB.btCollisionWorld_ConvexResultCallback_delete(self.ptr, ...)
	end
	function META:GetCollisionFilterMask(...)
		return CLIB.btCollisionWorld_ConvexResultCallback_getCollisionFilterMask(self.ptr, ...)
	end
	function META:HasHit(...)
		return CLIB.btCollisionWorld_ConvexResultCallback_hasHit(self.ptr, ...)
	end
	function META:SetCollisionFilterGroup(...)
		return CLIB.btCollisionWorld_ConvexResultCallback_setCollisionFilterGroup(self.ptr, ...)
	end
	function META:SetClosestHitFraction(...)
		return CLIB.btCollisionWorld_ConvexResultCallback_setClosestHitFraction(self.ptr, ...)
	end
end
do -- btBoxBoxCollisionAlgorithm_CreateFunc
	local META = {}
	library.metatables.btBoxBoxCollisionAlgorithm_CreateFunc = META
	META.__index = function(s, k) return META[k] end
	function library.CreateBoxBoxCollisionAlgorithm_CreateFunc(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btBoxBoxCollisionAlgorithm_CreateFunc_new(...)
		return self
	end
end
do -- btAlignedObjectArray_btSoftBody_Tetra
	local META = {}
	library.metatables.btAlignedObjectArray_btSoftBody_Tetra = META
	META.__index = function(s, k) return META[k] end
	function META:Size(...)
		return CLIB.btAlignedObjectArray_btSoftBody_Tetra_size(self.ptr, ...)
	end
	function META:ResizeNoInitialize(...)
		return CLIB.btAlignedObjectArray_btSoftBody_Tetra_resizeNoInitialize(self.ptr, ...)
	end
	function META:At(...)
		return CLIB.btAlignedObjectArray_btSoftBody_Tetra_at(self.ptr, ...)
	end
end
do -- btCompoundCompoundCollisionAlgorithm
	local META = {}
	library.metatables.btCompoundCompoundCollisionAlgorithm = META
	META.__index = function(s, k) return META[k] end
	function library.CreateCompoundCompoundCollisionAlgorithm(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btCompoundCompoundCollisionAlgorithm_new(...)
		return self
	end
end
do -- btAlignedObjectArray_btSoftBody_Link
	local META = {}
	library.metatables.btAlignedObjectArray_btSoftBody_Link = META
	META.__index = function(s, k) return META[k] end
	function META:Set(...)
		return CLIB.btAlignedObjectArray_btSoftBody_Link_set(self.ptr, ...)
	end
	function META:ResizeNoInitialize(...)
		return CLIB.btAlignedObjectArray_btSoftBody_Link_resizeNoInitialize(self.ptr, ...)
	end
	function META:At(...)
		return CLIB.btAlignedObjectArray_btSoftBody_Link_at(self.ptr, ...)
	end
	function META:Size(...)
		return CLIB.btAlignedObjectArray_btSoftBody_Link_size(self.ptr, ...)
	end
end
do -- btAlignedObjectArray_btSoftBody_Face
	local META = {}
	library.metatables.btAlignedObjectArray_btSoftBody_Face = META
	META.__index = function(s, k) return META[k] end
	function META:ResizeNoInitialize(...)
		return CLIB.btAlignedObjectArray_btSoftBody_Face_resizeNoInitialize(self.ptr, ...)
	end
	function META:Size(...)
		return CLIB.btAlignedObjectArray_btSoftBody_Face_size(self.ptr, ...)
	end
	function META:At(...)
		return CLIB.btAlignedObjectArray_btSoftBody_Face_at(self.ptr, ...)
	end
end
do -- btDiscreteCollisionDetectorInterface
	local META = {}
	library.metatables.btDiscreteCollisionDetectorInterface = META
	META.__index = function(s, k) return META[k] end
	function META:Delete(...)
		return CLIB.btDiscreteCollisionDetectorInterface_delete(self.ptr, ...)
	end
	function META:GetClosestPoints(...)
		return CLIB.btDiscreteCollisionDetectorInterface_getClosestPoints(self.ptr, ...)
	end
end
do -- btCollisionAlgorithmConstructionInfo
	local META = {}
	library.metatables.btCollisionAlgorithmConstructionInfo = META
	META.__index = function(s, k) return META[k] end
	function library.CreateCollisionAlgorithmConstructionInfo(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btCollisionAlgorithmConstructionInfo_new(...)
		return self
	end
	function library.CreateCollisionAlgorithmConstructionInfo2(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btCollisionAlgorithmConstructionInfo_new2(...)
		return self
	end
	function META:SetDispatcher1(...)
		return CLIB.btCollisionAlgorithmConstructionInfo_setDispatcher1(self.ptr, ...)
	end
	function META:GetManifold(...)
		return CLIB.btCollisionAlgorithmConstructionInfo_getManifold(self.ptr, ...)
	end
	function META:GetDispatcher1(...)
		return CLIB.btCollisionAlgorithmConstructionInfo_getDispatcher1(self.ptr, ...)
	end
	function META:SetManifold(...)
		return CLIB.btCollisionAlgorithmConstructionInfo_setManifold(self.ptr, ...)
	end
	function META:Delete(...)
		return CLIB.btCollisionAlgorithmConstructionInfo_delete(self.ptr, ...)
	end
end
do -- btPrimitiveManagerBase_get_primitive
	local META = {}
	library.metatables.btPrimitiveManagerBase_get_primitive = META
	META.__index = function(s, k) return META[k] end
	function META:Triangle(...)
		return CLIB.btPrimitiveManagerBase_get_primitive_triangle(self.ptr, ...)
	end
	function META:Box(...)
		return CLIB.btPrimitiveManagerBase_get_primitive_box(self.ptr, ...)
	end
	function META:Count(...)
		return CLIB.btPrimitiveManagerBase_get_primitive_count(self.ptr, ...)
	end
end
do -- btAlignedObjectArray_btSoftBody_Note
	local META = {}
	library.metatables.btAlignedObjectArray_btSoftBody_Note = META
	META.__index = function(s, k) return META[k] end
	function META:Size(...)
		return CLIB.btAlignedObjectArray_btSoftBody_Note_size(self.ptr, ...)
	end
	function META:At(...)
		return CLIB.btAlignedObjectArray_btSoftBody_Note_at(self.ptr, ...)
	end
end
do -- btAlignedObjectArray_btSoftBody_Node
	local META = {}
	library.metatables.btAlignedObjectArray_btSoftBody_Node = META
	META.__index = function(s, k) return META[k] end
	function META:Size(...)
		return CLIB.btAlignedObjectArray_btSoftBody_Node_size(self.ptr, ...)
	end
	function META:ResizeNoInitialize(...)
		return CLIB.btAlignedObjectArray_btSoftBody_Node_resizeNoInitialize(self.ptr, ...)
	end
	function META:At(...)
		return CLIB.btAlignedObjectArray_btSoftBody_Node_at(self.ptr, ...)
	end
end
do -- btSequentialImpulseConstraintSolver
	local META = {}
	library.metatables.btSequentialImpulseConstraintSolver = META
	META.__index = function(s, k) return META[k] end
	function library.CreateSequentialImpulseConstraintSolver(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btSequentialImpulseConstraintSolver_new(...)
		return self
	end
	function META:BtRandInt2(...)
		return CLIB.btSequentialImpulseConstraintSolver_btRandInt2(self.ptr, ...)
	end
	function META:BtRand2(...)
		return CLIB.btSequentialImpulseConstraintSolver_btRand2(self.ptr, ...)
	end
	function META:SetRandSeed(...)
		return CLIB.btSequentialImpulseConstraintSolver_setRandSeed(self.ptr, ...)
	end
	function META:GetRandSeed(...)
		return CLIB.btSequentialImpulseConstraintSolver_getRandSeed(self.ptr, ...)
	end
end
do -- btSoftBodyConcaveCollisionAlgorithm
	local META = {}
	library.metatables.btSoftBodyConcaveCollisionAlgorithm = META
	META.__index = function(s, k) return META[k] end
	function library.CreateSoftBodyConcaveCollisionAlgorithm(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btSoftBodyConcaveCollisionAlgorithm_new(...)
		return self
	end
	function META:ClearCache(...)
		return CLIB.btSoftBodyConcaveCollisionAlgorithm_clearCache(self.ptr, ...)
	end
end
do -- btTypedConstraint_btConstraintInfo2
	local META = {}
	library.metatables.btTypedConstraint_btConstraintInfo2 = META
	META.__index = function(s, k) return META[k] end
	function library.CreateTypedConstraint_btConstraintInfo2(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btTypedConstraint_btConstraintInfo2_new(...)
		return self
	end
	function META:GetErp(...)
		return CLIB.btTypedConstraint_btConstraintInfo2_getErp(self.ptr, ...)
	end
	function META:GetJ2linearAxis(...)
		return CLIB.btTypedConstraint_btConstraintInfo2_getJ2linearAxis(self.ptr, ...)
	end
	function META:SetRowskip(...)
		return CLIB.btTypedConstraint_btConstraintInfo2_setRowskip(self.ptr, ...)
	end
	function META:GetLowerLimit(...)
		return CLIB.btTypedConstraint_btConstraintInfo2_getLowerLimit(self.ptr, ...)
	end
	function META:Delete(...)
		return CLIB.btTypedConstraint_btConstraintInfo2_delete(self.ptr, ...)
	end
	function META:SetUpperLimit(...)
		return CLIB.btTypedConstraint_btConstraintInfo2_setUpperLimit(self.ptr, ...)
	end
	function META:SetNumIterations(...)
		return CLIB.btTypedConstraint_btConstraintInfo2_setNumIterations(self.ptr, ...)
	end
	function META:SetLowerLimit(...)
		return CLIB.btTypedConstraint_btConstraintInfo2_setLowerLimit(self.ptr, ...)
	end
	function META:SetJ2linearAxis(...)
		return CLIB.btTypedConstraint_btConstraintInfo2_setJ2linearAxis(self.ptr, ...)
	end
	function META:SetJ2angularAxis(...)
		return CLIB.btTypedConstraint_btConstraintInfo2_setJ2angularAxis(self.ptr, ...)
	end
	function META:SetJ1linearAxis(...)
		return CLIB.btTypedConstraint_btConstraintInfo2_setJ1linearAxis(self.ptr, ...)
	end
	function META:SetJ1angularAxis(...)
		return CLIB.btTypedConstraint_btConstraintInfo2_setJ1angularAxis(self.ptr, ...)
	end
	function META:SetFps(...)
		return CLIB.btTypedConstraint_btConstraintInfo2_setFps(self.ptr, ...)
	end
	function META:SetErp(...)
		return CLIB.btTypedConstraint_btConstraintInfo2_setErp(self.ptr, ...)
	end
	function META:SetDamping(...)
		return CLIB.btTypedConstraint_btConstraintInfo2_setDamping(self.ptr, ...)
	end
	function META:SetConstraintError(...)
		return CLIB.btTypedConstraint_btConstraintInfo2_setConstraintError(self.ptr, ...)
	end
	function META:SetCfm(...)
		return CLIB.btTypedConstraint_btConstraintInfo2_setCfm(self.ptr, ...)
	end
	function META:GetUpperLimit(...)
		return CLIB.btTypedConstraint_btConstraintInfo2_getUpperLimit(self.ptr, ...)
	end
	function META:GetJ2angularAxis(...)
		return CLIB.btTypedConstraint_btConstraintInfo2_getJ2angularAxis(self.ptr, ...)
	end
	function META:GetJ1angularAxis(...)
		return CLIB.btTypedConstraint_btConstraintInfo2_getJ1angularAxis(self.ptr, ...)
	end
	function META:GetFps(...)
		return CLIB.btTypedConstraint_btConstraintInfo2_getFps(self.ptr, ...)
	end
	function META:GetNumIterations(...)
		return CLIB.btTypedConstraint_btConstraintInfo2_getNumIterations(self.ptr, ...)
	end
	function META:GetRowskip(...)
		return CLIB.btTypedConstraint_btConstraintInfo2_getRowskip(self.ptr, ...)
	end
	function META:GetConstraintError(...)
		return CLIB.btTypedConstraint_btConstraintInfo2_getConstraintError(self.ptr, ...)
	end
	function META:GetCfm(...)
		return CLIB.btTypedConstraint_btConstraintInfo2_getCfm(self.ptr, ...)
	end
	function META:GetJ1linearAxis(...)
		return CLIB.btTypedConstraint_btConstraintInfo2_getJ1linearAxis(self.ptr, ...)
	end
	function META:GetDamping(...)
		return CLIB.btTypedConstraint_btConstraintInfo2_getDamping(self.ptr, ...)
	end
end
do -- btAlignedObjectArray_btVector3_push
	local META = {}
	library.metatables.btAlignedObjectArray_btVector3_push = META
	META.__index = function(s, k) return META[k] end
	function META:Back(...)
		return CLIB.btAlignedObjectArray_btVector3_push_back(self.ptr, ...)
	end
	function META:Back2(...)
		return CLIB.btAlignedObjectArray_btVector3_push_back2(self.ptr, ...)
	end
end
do -- btTypedConstraint_btConstraintInfo1
	local META = {}
	library.metatables.btTypedConstraint_btConstraintInfo1 = META
	META.__index = function(s, k) return META[k] end
	function library.CreateTypedConstraint_btConstraintInfo1(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btTypedConstraint_btConstraintInfo1_new(...)
		return self
	end
	function META:GetNub(...)
		return CLIB.btTypedConstraint_btConstraintInfo1_getNub(self.ptr, ...)
	end
	function META:SetNumConstraintRows(...)
		return CLIB.btTypedConstraint_btConstraintInfo1_setNumConstraintRows(self.ptr, ...)
	end
	function META:SetNub(...)
		return CLIB.btTypedConstraint_btConstraintInfo1_setNub(self.ptr, ...)
	end
	function META:GetNumConstraintRows(...)
		return CLIB.btTypedConstraint_btConstraintInfo1_getNumConstraintRows(self.ptr, ...)
	end
	function META:Delete(...)
		return CLIB.btTypedConstraint_btConstraintInfo1_delete(self.ptr, ...)
	end
end
do -- btConvexConvexAlgorithm_CreateFunc
	local META = {}
	library.metatables.btConvexConvexAlgorithm_CreateFunc = META
	META.__index = function(s, k) return META[k] end
	function library.CreateConvexConvexAlgorithm_CreateFunc(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btConvexConvexAlgorithm_CreateFunc_new(...)
		return self
	end
	function META:GetPdSolver(...)
		return CLIB.btConvexConvexAlgorithm_CreateFunc_getPdSolver(self.ptr, ...)
	end
	function META:SetMinimumPointsPerturbationThreshold(...)
		return CLIB.btConvexConvexAlgorithm_CreateFunc_setMinimumPointsPerturbationThreshold(self.ptr, ...)
	end
	function META:SetPdSolver(...)
		return CLIB.btConvexConvexAlgorithm_CreateFunc_setPdSolver(self.ptr, ...)
	end
	function META:GetNumPerturbationIterations(...)
		return CLIB.btConvexConvexAlgorithm_CreateFunc_getNumPerturbationIterations(self.ptr, ...)
	end
	function META:GetMinimumPointsPerturbationThreshold(...)
		return CLIB.btConvexConvexAlgorithm_CreateFunc_getMinimumPointsPerturbationThreshold(self.ptr, ...)
	end
	function META:SetNumPerturbationIterations(...)
		return CLIB.btConvexConvexAlgorithm_CreateFunc_setNumPerturbationIterations(self.ptr, ...)
	end
end
do -- btAlignedObjectArray_btIndexedMesh
	local META = {}
	library.metatables.btAlignedObjectArray_btIndexedMesh = META
	META.__index = function(s, k) return META[k] end
	function META:Size(...)
		return CLIB.btAlignedObjectArray_btIndexedMesh_size(self.ptr, ...)
	end
	function META:At(...)
		return CLIB.btAlignedObjectArray_btIndexedMesh_at(self.ptr, ...)
	end
	function META:ResizeNoInitialize(...)
		return CLIB.btAlignedObjectArray_btIndexedMesh_resizeNoInitialize(self.ptr, ...)
	end
end
do -- btCollisionWorld_RayResultCallback
	local META = {}
	library.metatables.btCollisionWorld_RayResultCallback = META
	META.__index = function(s, k) return META[k] end
	function META:SetCollisionFilterMask(...)
		return CLIB.btCollisionWorld_RayResultCallback_setCollisionFilterMask(self.ptr, ...)
	end
	function META:SetFlags(...)
		return CLIB.btCollisionWorld_RayResultCallback_setFlags(self.ptr, ...)
	end
	function META:SetCollisionFilterGroup(...)
		return CLIB.btCollisionWorld_RayResultCallback_setCollisionFilterGroup(self.ptr, ...)
	end
	function META:GetCollisionObject(...)
		return CLIB.btCollisionWorld_RayResultCallback_getCollisionObject(self.ptr, ...)
	end
	function META:SetCollisionObject(...)
		return CLIB.btCollisionWorld_RayResultCallback_setCollisionObject(self.ptr, ...)
	end
	function META:GetClosestHitFraction(...)
		return CLIB.btCollisionWorld_RayResultCallback_getClosestHitFraction(self.ptr, ...)
	end
	function META:Delete(...)
		return CLIB.btCollisionWorld_RayResultCallback_delete(self.ptr, ...)
	end
	function META:SetClosestHitFraction(...)
		return CLIB.btCollisionWorld_RayResultCallback_setClosestHitFraction(self.ptr, ...)
	end
	function META:GetCollisionFilterGroup(...)
		return CLIB.btCollisionWorld_RayResultCallback_getCollisionFilterGroup(self.ptr, ...)
	end
	function META:NeedsCollision(...)
		return CLIB.btCollisionWorld_RayResultCallback_needsCollision(self.ptr, ...)
	end
	function META:HasHit(...)
		return CLIB.btCollisionWorld_RayResultCallback_hasHit(self.ptr, ...)
	end
	function META:AddSingleResult(...)
		return CLIB.btCollisionWorld_RayResultCallback_addSingleResult(self.ptr, ...)
	end
	function META:GetFlags(...)
		return CLIB.btCollisionWorld_RayResultCallback_getFlags(self.ptr, ...)
	end
	function META:GetCollisionFilterMask(...)
		return CLIB.btCollisionWorld_RayResultCallback_getCollisionFilterMask(self.ptr, ...)
	end
end
do -- btPolyhedralConvexAabbCachingShape
	local META = {}
	library.metatables.btPolyhedralConvexAabbCachingShape = META
	META.__index = function(s, k) return META[k] end
	function META:RecalcLocalAabb(...)
		return CLIB.btPolyhedralConvexAabbCachingShape_recalcLocalAabb(self.ptr, ...)
	end
	function META:GetNonvirtualAabb(...)
		return CLIB.btPolyhedralConvexAabbCachingShape_getNonvirtualAabb(self.ptr, ...)
	end
end
do -- btTriangleIndexVertexMaterialArray
	local META = {}
	library.metatables.btTriangleIndexVertexMaterialArray = META
	META.__index = function(s, k) return META[k] end
	function library.CreateTriangleIndexVertexMaterialArray(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btTriangleIndexVertexMaterialArray_new2(...)
		return self
	end
	function library.CreateTriangleIndexVertexMaterialArray2(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btTriangleIndexVertexMaterialArray_new(...)
		return self
	end
	function META:AddMaterialProperties(...)
		return CLIB.btTriangleIndexVertexMaterialArray_addMaterialProperties(self.ptr, ...)
	end
	function META:GetLockedMaterialBase(...)
		return CLIB.btTriangleIndexVertexMaterialArray_getLockedMaterialBase(self.ptr, ...)
	end
	function META:GetLockedReadOnlyMaterialBase(...)
		return CLIB.btTriangleIndexVertexMaterialArray_getLockedReadOnlyMaterialBase(self.ptr, ...)
	end
end
do -- btCollisionWorld_LocalConvexResult
	local META = {}
	library.metatables.btCollisionWorld_LocalConvexResult = META
	META.__index = function(s, k) return META[k] end
	function library.CreateCollisionWorld_LocalConvexResult(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btCollisionWorld_LocalConvexResult_new(...)
		return self
	end
	function META:GetHitCollisionObject(...)
		return CLIB.btCollisionWorld_LocalConvexResult_getHitCollisionObject(self.ptr, ...)
	end
	function META:Delete(...)
		return CLIB.btCollisionWorld_LocalConvexResult_delete(self.ptr, ...)
	end
	function META:SetLocalShapeInfo(...)
		return CLIB.btCollisionWorld_LocalConvexResult_setLocalShapeInfo(self.ptr, ...)
	end
	function META:GetLocalShapeInfo(...)
		return CLIB.btCollisionWorld_LocalConvexResult_getLocalShapeInfo(self.ptr, ...)
	end
	function META:SetHitNormalLocal(...)
		return CLIB.btCollisionWorld_LocalConvexResult_setHitNormalLocal(self.ptr, ...)
	end
	function META:SetHitFraction(...)
		return CLIB.btCollisionWorld_LocalConvexResult_setHitFraction(self.ptr, ...)
	end
	function META:SetHitPointLocal(...)
		return CLIB.btCollisionWorld_LocalConvexResult_setHitPointLocal(self.ptr, ...)
	end
	function META:GetHitFraction(...)
		return CLIB.btCollisionWorld_LocalConvexResult_getHitFraction(self.ptr, ...)
	end
	function META:GetHitNormalLocal(...)
		return CLIB.btCollisionWorld_LocalConvexResult_getHitNormalLocal(self.ptr, ...)
	end
	function META:GetHitPointLocal(...)
		return CLIB.btCollisionWorld_LocalConvexResult_getHitPointLocal(self.ptr, ...)
	end
	function META:SetHitCollisionObject(...)
		return CLIB.btCollisionWorld_LocalConvexResult_setHitCollisionObject(self.ptr, ...)
	end
end
do -- btSphereTriangleCollisionAlgorithm
	local META = {}
	library.metatables.btSphereTriangleCollisionAlgorithm = META
	META.__index = function(s, k) return META[k] end
	function library.CreateSphereTriangleCollisionAlgorithm(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btSphereTriangleCollisionAlgorithm_new(...)
		return self
	end
	function library.CreateSphereTriangleCollisionAlgorithm2(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btSphereTriangleCollisionAlgorithm_new2(...)
		return self
	end
end
do -- btDefaultCollisionConstructionInfo
	local META = {}
	library.metatables.btDefaultCollisionConstructionInfo = META
	META.__index = function(s, k) return META[k] end
	function library.CreateDefaultCollisionConstructionInfo(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btDefaultCollisionConstructionInfo_new(...)
		return self
	end
	function META:SetUseEpaPenetrationAlgorithm(...)
		return CLIB.btDefaultCollisionConstructionInfo_setUseEpaPenetrationAlgorithm(self.ptr, ...)
	end
	function META:GetDefaultMaxPersistentManifoldPoolSize(...)
		return CLIB.btDefaultCollisionConstructionInfo_getDefaultMaxPersistentManifoldPoolSize(self.ptr, ...)
	end
	function META:SetPersistentManifoldPool(...)
		return CLIB.btDefaultCollisionConstructionInfo_setPersistentManifoldPool(self.ptr, ...)
	end
	function META:SetCustomCollisionAlgorithmMaxElementSize(...)
		return CLIB.btDefaultCollisionConstructionInfo_setCustomCollisionAlgorithmMaxElementSize(self.ptr, ...)
	end
	function META:Delete(...)
		return CLIB.btDefaultCollisionConstructionInfo_delete(self.ptr, ...)
	end
	function META:SetCollisionAlgorithmPool(...)
		return CLIB.btDefaultCollisionConstructionInfo_setCollisionAlgorithmPool(self.ptr, ...)
	end
	function META:GetPersistentManifoldPool(...)
		return CLIB.btDefaultCollisionConstructionInfo_getPersistentManifoldPool(self.ptr, ...)
	end
	function META:GetCustomCollisionAlgorithmMaxElementSize(...)
		return CLIB.btDefaultCollisionConstructionInfo_getCustomCollisionAlgorithmMaxElementSize(self.ptr, ...)
	end
	function META:GetDefaultMaxCollisionAlgorithmPoolSize(...)
		return CLIB.btDefaultCollisionConstructionInfo_getDefaultMaxCollisionAlgorithmPoolSize(self.ptr, ...)
	end
	function META:SetDefaultMaxCollisionAlgorithmPoolSize(...)
		return CLIB.btDefaultCollisionConstructionInfo_setDefaultMaxCollisionAlgorithmPoolSize(self.ptr, ...)
	end
	function META:GetUseEpaPenetrationAlgorithm(...)
		return CLIB.btDefaultCollisionConstructionInfo_getUseEpaPenetrationAlgorithm(self.ptr, ...)
	end
	function META:GetCollisionAlgorithmPool(...)
		return CLIB.btDefaultCollisionConstructionInfo_getCollisionAlgorithmPool(self.ptr, ...)
	end
	function META:SetDefaultMaxPersistentManifoldPoolSize(...)
		return CLIB.btDefaultCollisionConstructionInfo_setDefaultMaxPersistentManifoldPoolSize(self.ptr, ...)
	end
end
do -- btAlignedObjectArray_btSoftBodyPtr
	local META = {}
	library.metatables.btAlignedObjectArray_btSoftBodyPtr = META
	META.__index = function(s, k) return META[k] end
	function META:ResizeNoInitialize(...)
		return CLIB.btAlignedObjectArray_btSoftBodyPtr_resizeNoInitialize(self.ptr, ...)
	end
	function META:At(...)
		return CLIB.btAlignedObjectArray_btSoftBodyPtr_at(self.ptr, ...)
	end
	function META:Size(...)
		return CLIB.btAlignedObjectArray_btSoftBodyPtr_size(self.ptr, ...)
	end
end
do -- btConvexConcaveCollisionAlgorithm
	local META = {}
	library.metatables.btConvexConcaveCollisionAlgorithm = META
	META.__index = function(s, k) return META[k] end
	function library.CreateConvexConcaveCollisionAlgorithm(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btConvexConcaveCollisionAlgorithm_new(...)
		return self
	end
	function META:ClearCache(...)
		return CLIB.btConvexConcaveCollisionAlgorithm_clearCache(self.ptr, ...)
	end
end
do -- btBroadphaseRayCallback_setLambda
	local META = {}
	library.metatables.btBroadphaseRayCallback_setLambda = META
	META.__index = function(s, k) return META[k] end
	function META:Max(...)
		return CLIB.btBroadphaseRayCallback_setLambda_max(self.ptr, ...)
	end
end
do -- btHingeAccumulatedAngleConstraint
	local META = {}
	library.metatables.btHingeAccumulatedAngleConstraint = META
	META.__index = function(s, k) return META[k] end
	function library.CreateHingeAccumulatedAngleConstraint(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btHingeAccumulatedAngleConstraint_new(...)
		return self
	end
	function library.CreateHingeAccumulatedAngleConstraint2(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btHingeAccumulatedAngleConstraint_new2(...)
		return self
	end
	function library.CreateHingeAccumulatedAngleConstraint3(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btHingeAccumulatedAngleConstraint_new4(...)
		return self
	end
	function library.CreateHingeAccumulatedAngleConstraint4(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btHingeAccumulatedAngleConstraint_new3(...)
		return self
	end
	function META:GetAccumulatedHingeAngle(...)
		return CLIB.btHingeAccumulatedAngleConstraint_getAccumulatedHingeAngle(self.ptr, ...)
	end
	function META:SetAccumulatedHingeAngle(...)
		return CLIB.btHingeAccumulatedAngleConstraint_setAccumulatedHingeAngle(self.ptr, ...)
	end
end
do -- btBroadphaseRayCallback_getLambda
	local META = {}
	library.metatables.btBroadphaseRayCallback_getLambda = META
	META.__index = function(s, k) return META[k] end
	function META:Max(...)
		return CLIB.btBroadphaseRayCallback_getLambda_max(self.ptr, ...)
	end
end
do -- btSoftBody_AJoint_IControlWrapper
	local META = {}
	library.metatables.btSoftBody_AJoint_IControlWrapper = META
	META.__index = function(s, k) return META[k] end
	function library.CreateSoftBody_AJoint_IControlWrapper(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btSoftBody_AJoint_IControlWrapper_new(...)
		return self
	end
	function META:SetWrapperData(...)
		return CLIB.btSoftBody_AJoint_IControlWrapper_setWrapperData(self.ptr, ...)
	end
	function META:GetWrapperData(...)
		return CLIB.btSoftBody_AJoint_IControlWrapper_getWrapperData(self.ptr, ...)
	end
end
do -- btMinkowskiPenetrationDepthSolver
	local META = {}
	library.metatables.btMinkowskiPenetrationDepthSolver = META
	META.__index = function(s, k) return META[k] end
	function library.CreateMinkowskiPenetrationDepthSolver(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btMinkowskiPenetrationDepthSolver_new(...)
		return self
	end
end
do -- btRaycastVehicle_btVehicleTuning
	local META = {}
	library.metatables.btRaycastVehicle_btVehicleTuning = META
	META.__index = function(s, k) return META[k] end
	function library.CreateRaycastVehicle_btVehicleTuning(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btRaycastVehicle_btVehicleTuning_new(...)
		return self
	end
	function META:GetSuspensionCompression(...)
		return CLIB.btRaycastVehicle_btVehicleTuning_getSuspensionCompression(self.ptr, ...)
	end
	function META:GetFrictionSlip(...)
		return CLIB.btRaycastVehicle_btVehicleTuning_getFrictionSlip(self.ptr, ...)
	end
	function META:Delete(...)
		return CLIB.btRaycastVehicle_btVehicleTuning_delete(self.ptr, ...)
	end
	function META:GetSuspensionDamping(...)
		return CLIB.btRaycastVehicle_btVehicleTuning_getSuspensionDamping(self.ptr, ...)
	end
	function META:SetSuspensionStiffness(...)
		return CLIB.btRaycastVehicle_btVehicleTuning_setSuspensionStiffness(self.ptr, ...)
	end
	function META:SetSuspensionDamping(...)
		return CLIB.btRaycastVehicle_btVehicleTuning_setSuspensionDamping(self.ptr, ...)
	end
	function META:SetSuspensionCompression(...)
		return CLIB.btRaycastVehicle_btVehicleTuning_setSuspensionCompression(self.ptr, ...)
	end
	function META:SetMaxSuspensionTravelCm(...)
		return CLIB.btRaycastVehicle_btVehicleTuning_setMaxSuspensionTravelCm(self.ptr, ...)
	end
	function META:SetMaxSuspensionForce(...)
		return CLIB.btRaycastVehicle_btVehicleTuning_setMaxSuspensionForce(self.ptr, ...)
	end
	function META:SetFrictionSlip(...)
		return CLIB.btRaycastVehicle_btVehicleTuning_setFrictionSlip(self.ptr, ...)
	end
	function META:GetMaxSuspensionTravelCm(...)
		return CLIB.btRaycastVehicle_btVehicleTuning_getMaxSuspensionTravelCm(self.ptr, ...)
	end
	function META:GetMaxSuspensionForce(...)
		return CLIB.btRaycastVehicle_btVehicleTuning_getMaxSuspensionForce(self.ptr, ...)
	end
	function META:GetSuspensionStiffness(...)
		return CLIB.btRaycastVehicle_btVehicleTuning_getSuspensionStiffness(self.ptr, ...)
	end
end
do -- btMultimaterialTriangleMeshShape
	local META = {}
	library.metatables.btMultimaterialTriangleMeshShape = META
	META.__index = function(s, k) return META[k] end
	function library.CreateMultimaterialTriangleMeshShape(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btMultimaterialTriangleMeshShape_new2(...)
		return self
	end
	function library.CreateMultimaterialTriangleMeshShape2(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btMultimaterialTriangleMeshShape_new(...)
		return self
	end
	function META:GetMaterialProperties(...)
		return CLIB.btMultimaterialTriangleMeshShape_getMaterialProperties(self.ptr, ...)
	end
end
do -- btSphereSphereCollisionAlgorithm
	local META = {}
	library.metatables.btSphereSphereCollisionAlgorithm = META
	META.__index = function(s, k) return META[k] end
	function library.CreateSphereSphereCollisionAlgorithm(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btSphereSphereCollisionAlgorithm_new2(...)
		return self
	end
	function library.CreateSphereSphereCollisionAlgorithm2(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btSphereSphereCollisionAlgorithm_new(...)
		return self
	end
end
do -- btPrimitiveTriangle_overlap_test
	local META = {}
	library.metatables.btPrimitiveTriangle_overlap_test = META
	META.__index = function(s, k) return META[k] end
	function META:Conservative(...)
		return CLIB.btPrimitiveTriangle_overlap_test_conservative(self.ptr, ...)
	end
end
do -- btConvexInternalAabbCachingShape
	local META = {}
	library.metatables.btConvexInternalAabbCachingShape = META
	META.__index = function(s, k) return META[k] end
	function META:RecalcLocalAabb(...)
		return CLIB.btConvexInternalAabbCachingShape_recalcLocalAabb(self.ptr, ...)
	end
end
do -- btDefaultCollisionConfiguration
	local META = {}
	library.metatables.btDefaultCollisionConfiguration = META
	META.__index = function(s, k) return META[k] end
	function library.CreateDefaultCollisionConfiguration(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btDefaultCollisionConfiguration_new(...)
		return self
	end
	function library.CreateDefaultCollisionConfiguration2(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btDefaultCollisionConfiguration_new2(...)
		return self
	end
	function META:SetConvexConvexMultipointIterations(...)
		return CLIB.btDefaultCollisionConfiguration_setConvexConvexMultipointIterations(self.ptr, ...)
	end
	function META:GetClosestPointsAlgorithmCreateFunc(...)
		return CLIB.btDefaultCollisionConfiguration_getClosestPointsAlgorithmCreateFunc(self.ptr, ...)
	end
	function META:SetPlaneConvexMultipointIterations(...)
		return CLIB.btDefaultCollisionConfiguration_setPlaneConvexMultipointIterations(self.ptr, ...)
	end
end
do -- btMultiBodyJointLimitConstraint
	local META = {}
	library.metatables.btMultiBodyJointLimitConstraint = META
	META.__index = function(s, k) return META[k] end
	function library.CreateMultiBodyJointLimitConstraint(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btMultiBodyJointLimitConstraint_new(...)
		return self
	end
end
do -- btBroadphaseAabbCallbackWrapper
	local META = {}
	library.metatables.btBroadphaseAabbCallbackWrapper = META
	META.__index = function(s, k) return META[k] end
	function library.CreateBroadphaseAabbCallbackWrapper(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btBroadphaseAabbCallbackWrapper_new(...)
		return self
	end
end
do -- btInternalTriangleIndexCallback
	local META = {}
	library.metatables.btInternalTriangleIndexCallback = META
	META.__index = function(s, k) return META[k] end
	function META:Delete(...)
		return CLIB.btInternalTriangleIndexCallback_delete(self.ptr, ...)
	end
end
do -- btCollisionWorld_LocalShapeInfo
	local META = {}
	library.metatables.btCollisionWorld_LocalShapeInfo = META
	META.__index = function(s, k) return META[k] end
	function library.CreateCollisionWorld_LocalShapeInfo(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btCollisionWorld_LocalShapeInfo_new(...)
		return self
	end
	function META:GetShapePart(...)
		return CLIB.btCollisionWorld_LocalShapeInfo_getShapePart(self.ptr, ...)
	end
	function META:SetTriangleIndex(...)
		return CLIB.btCollisionWorld_LocalShapeInfo_setTriangleIndex(self.ptr, ...)
	end
	function META:GetTriangleIndex(...)
		return CLIB.btCollisionWorld_LocalShapeInfo_getTriangleIndex(self.ptr, ...)
	end
	function META:Delete(...)
		return CLIB.btCollisionWorld_LocalShapeInfo_delete(self.ptr, ...)
	end
	function META:SetShapePart(...)
		return CLIB.btCollisionWorld_LocalShapeInfo_setShapePart(self.ptr, ...)
	end
end
do -- btCollisionWorld_LocalRayResult
	local META = {}
	library.metatables.btCollisionWorld_LocalRayResult = META
	META.__index = function(s, k) return META[k] end
	function library.CreateCollisionWorld_LocalRayResult(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btCollisionWorld_LocalRayResult_new(...)
		return self
	end
	function META:GetHitNormalLocal(...)
		return CLIB.btCollisionWorld_LocalRayResult_getHitNormalLocal(self.ptr, ...)
	end
	function META:GetHitFraction(...)
		return CLIB.btCollisionWorld_LocalRayResult_getHitFraction(self.ptr, ...)
	end
	function META:GetCollisionObject(...)
		return CLIB.btCollisionWorld_LocalRayResult_getCollisionObject(self.ptr, ...)
	end
	function META:Delete(...)
		return CLIB.btCollisionWorld_LocalRayResult_delete(self.ptr, ...)
	end
	function META:GetLocalShapeInfo(...)
		return CLIB.btCollisionWorld_LocalRayResult_getLocalShapeInfo(self.ptr, ...)
	end
	function META:SetHitFraction(...)
		return CLIB.btCollisionWorld_LocalRayResult_setHitFraction(self.ptr, ...)
	end
	function META:SetHitNormalLocal(...)
		return CLIB.btCollisionWorld_LocalRayResult_setHitNormalLocal(self.ptr, ...)
	end
	function META:SetCollisionObject(...)
		return CLIB.btCollisionWorld_LocalRayResult_setCollisionObject(self.ptr, ...)
	end
	function META:SetLocalShapeInfo(...)
		return CLIB.btCollisionWorld_LocalRayResult_setLocalShapeInfo(self.ptr, ...)
	end
end
do -- btConvexPlaneCollisionAlgorithm
	local META = {}
	library.metatables.btConvexPlaneCollisionAlgorithm = META
	META.__index = function(s, k) return META[k] end
	function library.CreateConvexPlaneCollisionAlgorithm(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btConvexPlaneCollisionAlgorithm_new(...)
		return self
	end
	function META:CollideSingleContact(...)
		return CLIB.btConvexPlaneCollisionAlgorithm_collideSingleContact(self.ptr, ...)
	end
end
do -- btConvexPenetrationDepthSolver
	local META = {}
	library.metatables.btConvexPenetrationDepthSolver = META
	META.__index = function(s, k) return META[k] end
	function META:CalcPenDepth(...)
		return CLIB.btConvexPenetrationDepthSolver_calcPenDepth(self.ptr, ...)
	end
	function META:Delete(...)
		return CLIB.btConvexPenetrationDepthSolver_delete(self.ptr, ...)
	end
end
do -- btOverlapFilterCallbackWrapper
	local META = {}
	library.metatables.btOverlapFilterCallbackWrapper = META
	META.__index = function(s, k) return META[k] end
	function library.CreateOverlapFilterCallbackWrapper(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btOverlapFilterCallbackWrapper_new(...)
		return self
	end
end
do -- btBox2dBox2dCollisionAlgorithm
	local META = {}
	library.metatables.btBox2dBox2dCollisionAlgorithm = META
	META.__index = function(s, k) return META[k] end
	function library.CreateBox2dBox2dCollisionAlgorithm(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btBox2dBox2dCollisionAlgorithm_new2(...)
		return self
	end
	function library.CreateBox2dBox2dCollisionAlgorithm2(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btBox2dBox2dCollisionAlgorithm_new(...)
		return self
	end
end
do -- btGeneric6DofSpring2Constraint
	local META = {}
	library.metatables.btGeneric6DofSpring2Constraint = META
	META.__index = function(s, k) return META[k] end
	function library.CreateGeneric6DofSpring2Constraint(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btGeneric6DofSpring2Constraint_new(...)
		return self
	end
	function library.CreateGeneric6DofSpring2Constraint2(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btGeneric6DofSpring2Constraint_new2(...)
		return self
	end
	function META:SetLimitReversed(...)
		return CLIB.btGeneric6DofSpring2Constraint_setLimitReversed(self.ptr, ...)
	end
	function META:GetLinearLowerLimit(...)
		return CLIB.btGeneric6DofSpring2Constraint_getLinearLowerLimit(self.ptr, ...)
	end
	function META:SetFrames(...)
		return CLIB.btGeneric6DofSpring2Constraint_setFrames(self.ptr, ...)
	end
	function META:GetAxis(...)
		return CLIB.btGeneric6DofSpring2Constraint_getAxis(self.ptr, ...)
	end
	function META:EnableSpring(...)
		return CLIB.btGeneric6DofSpring2Constraint_enableSpring(self.ptr, ...)
	end
	function META:GetRotationalLimitMotor(...)
		return CLIB.btGeneric6DofSpring2Constraint_getRotationalLimitMotor(self.ptr, ...)
	end
	function META:SetAngularLowerLimitReversed(...)
		return CLIB.btGeneric6DofSpring2Constraint_setAngularLowerLimitReversed(self.ptr, ...)
	end
	function META:IsLimited(...)
		return CLIB.btGeneric6DofSpring2Constraint_isLimited(self.ptr, ...)
	end
	function META:CalculateTransforms(...)
		return CLIB.btGeneric6DofSpring2Constraint_calculateTransforms(self.ptr, ...)
	end
	function META:MatrixToEulerXZY(...)
		return CLIB.btGeneric6DofSpring2Constraint_matrixToEulerXZY(self.ptr, ...)
	end
	function META:SetLinearUpperLimit(...)
		return CLIB.btGeneric6DofSpring2Constraint_setLinearUpperLimit(self.ptr, ...)
	end
	function META:SetServoTarget(...)
		return CLIB.btGeneric6DofSpring2Constraint_setServoTarget(self.ptr, ...)
	end
	function META:SetServo(...)
		return CLIB.btGeneric6DofSpring2Constraint_setServo(self.ptr, ...)
	end
	function META:MatrixToEulerYXZ(...)
		return CLIB.btGeneric6DofSpring2Constraint_matrixToEulerYXZ(self.ptr, ...)
	end
	function META:MatrixToEulerZYX(...)
		return CLIB.btGeneric6DofSpring2Constraint_matrixToEulerZYX(self.ptr, ...)
	end
	function META:CalculateTransforms2(...)
		return CLIB.btGeneric6DofSpring2Constraint_calculateTransforms2(self.ptr, ...)
	end
	function META:GetTranslationalLimitMotor(...)
		return CLIB.btGeneric6DofSpring2Constraint_getTranslationalLimitMotor(self.ptr, ...)
	end
	function META:GetRelativePivotPosition(...)
		return CLIB.btGeneric6DofSpring2Constraint_getRelativePivotPosition(self.ptr, ...)
	end
	function META:GetRotationOrder(...)
		return CLIB.btGeneric6DofSpring2Constraint_getRotationOrder(self.ptr, ...)
	end
	function META:GetAngularLowerLimit(...)
		return CLIB.btGeneric6DofSpring2Constraint_getAngularLowerLimit(self.ptr, ...)
	end
	function META:SetDamping(...)
		return CLIB.btGeneric6DofSpring2Constraint_setDamping(self.ptr, ...)
	end
	function META:GetFrameOffsetA(...)
		return CLIB.btGeneric6DofSpring2Constraint_getFrameOffsetA(self.ptr, ...)
	end
	function META:SetMaxMotorForce(...)
		return CLIB.btGeneric6DofSpring2Constraint_setMaxMotorForce(self.ptr, ...)
	end
	function META:EnableMotor(...)
		return CLIB.btGeneric6DofSpring2Constraint_enableMotor(self.ptr, ...)
	end
	function META:SetEquilibriumPoint2(...)
		return CLIB.btGeneric6DofSpring2Constraint_setEquilibriumPoint2(self.ptr, ...)
	end
	function META:MatrixToEulerZXY(...)
		return CLIB.btGeneric6DofSpring2Constraint_matrixToEulerZXY(self.ptr, ...)
	end
	function META:SetStiffness(...)
		return CLIB.btGeneric6DofSpring2Constraint_setStiffness(self.ptr, ...)
	end
	function META:SetLimit(...)
		return CLIB.btGeneric6DofSpring2Constraint_setLimit(self.ptr, ...)
	end
	function META:MatrixToEulerYZX(...)
		return CLIB.btGeneric6DofSpring2Constraint_matrixToEulerYZX(self.ptr, ...)
	end
	function META:MatrixToEulerXYZ(...)
		return CLIB.btGeneric6DofSpring2Constraint_matrixToEulerXYZ(self.ptr, ...)
	end
	function META:SetAngularLowerLimit(...)
		return CLIB.btGeneric6DofSpring2Constraint_setAngularLowerLimit(self.ptr, ...)
	end
	function META:GetCalculatedTransformB(...)
		return CLIB.btGeneric6DofSpring2Constraint_getCalculatedTransformB(self.ptr, ...)
	end
	function META:GetFrameOffsetB(...)
		return CLIB.btGeneric6DofSpring2Constraint_getFrameOffsetB(self.ptr, ...)
	end
	function META:SetBounce(...)
		return CLIB.btGeneric6DofSpring2Constraint_setBounce(self.ptr, ...)
	end
	function META:GetAngle(...)
		return CLIB.btGeneric6DofSpring2Constraint_getAngle(self.ptr, ...)
	end
	function META:GetAngularLowerLimitReversed(...)
		return CLIB.btGeneric6DofSpring2Constraint_getAngularLowerLimitReversed(self.ptr, ...)
	end
	function META:GetAngularUpperLimit(...)
		return CLIB.btGeneric6DofSpring2Constraint_getAngularUpperLimit(self.ptr, ...)
	end
	function META:GetAngularUpperLimitReversed(...)
		return CLIB.btGeneric6DofSpring2Constraint_getAngularUpperLimitReversed(self.ptr, ...)
	end
	function META:SetEquilibriumPoint(...)
		return CLIB.btGeneric6DofSpring2Constraint_setEquilibriumPoint(self.ptr, ...)
	end
	function META:GetLinearUpperLimit(...)
		return CLIB.btGeneric6DofSpring2Constraint_getLinearUpperLimit(self.ptr, ...)
	end
	function META:SetAngularUpperLimit(...)
		return CLIB.btGeneric6DofSpring2Constraint_setAngularUpperLimit(self.ptr, ...)
	end
	function META:SetAngularUpperLimitReversed(...)
		return CLIB.btGeneric6DofSpring2Constraint_setAngularUpperLimitReversed(self.ptr, ...)
	end
	function META:SetEquilibriumPoint3(...)
		return CLIB.btGeneric6DofSpring2Constraint_setEquilibriumPoint3(self.ptr, ...)
	end
	function META:SetLinearLowerLimit(...)
		return CLIB.btGeneric6DofSpring2Constraint_setLinearLowerLimit(self.ptr, ...)
	end
	function META:GetCalculatedTransformA(...)
		return CLIB.btGeneric6DofSpring2Constraint_getCalculatedTransformA(self.ptr, ...)
	end
	function META:SetTargetVelocity(...)
		return CLIB.btGeneric6DofSpring2Constraint_setTargetVelocity(self.ptr, ...)
	end
	function META:SetRotationOrder(...)
		return CLIB.btGeneric6DofSpring2Constraint_setRotationOrder(self.ptr, ...)
	end
	function META:SetAxis(...)
		return CLIB.btGeneric6DofSpring2Constraint_setAxis(self.ptr, ...)
	end
	function META:BtGetMatrixElem(...)
		return CLIB.btGeneric6DofSpring2Constraint_btGetMatrixElem(self.ptr, ...)
	end
end
do -- btGImpactQuantizedBvh_get_node
	local META = {}
	library.metatables.btGImpactQuantizedBvh_get_node = META
	META.__index = function(s, k) return META[k] end
	function META:Pointer(...)
		return CLIB.btGImpactQuantizedBvh_get_node_pointer(self.ptr, ...)
	end
end
do -- btGjkEpaPenetrationDepthSolver
	local META = {}
	library.metatables.btGjkEpaPenetrationDepthSolver = META
	META.__index = function(s, k) return META[k] end
	function library.CreateGjkEpaPenetrationDepthSolver(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btGjkEpaPenetrationDepthSolver_new(...)
		return self
	end
end
do -- btAlignedObjectArray_btVector3
	local META = {}
	library.metatables.btAlignedObjectArray_btVector3 = META
	META.__index = function(s, k) return META[k] end
	function library.CreateAlignedObjectArray_btVector3(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btAlignedObjectArray_btVector3_new(...)
		return self
	end
	function META:Delete(...)
		return CLIB.btAlignedObjectArray_btVector3_delete(self.ptr, ...)
	end
	function META:Size(...)
		return CLIB.btAlignedObjectArray_btVector3_size(self.ptr, ...)
	end
	function META:Set(...)
		return CLIB.btAlignedObjectArray_btVector3_set(self.ptr, ...)
	end
	function META:At(...)
		return CLIB.btAlignedObjectArray_btVector3_at(self.ptr, ...)
	end
end
do -- btCharacterControllerInterface
	local META = {}
	library.metatables.btCharacterControllerInterface = META
	META.__index = function(s, k) return META[k] end
	function META:SetUpInterpolate(...)
		return CLIB.btCharacterControllerInterface_setUpInterpolate(self.ptr, ...)
	end
	function META:PreStep(...)
		return CLIB.btCharacterControllerInterface_preStep(self.ptr, ...)
	end
	function META:SetVelocityForTimeInterval(...)
		return CLIB.btCharacterControllerInterface_setVelocityForTimeInterval(self.ptr, ...)
	end
	function META:Warp(...)
		return CLIB.btCharacterControllerInterface_warp(self.ptr, ...)
	end
	function META:SetWalkDirection(...)
		return CLIB.btCharacterControllerInterface_setWalkDirection(self.ptr, ...)
	end
	function META:PlayerStep(...)
		return CLIB.btCharacterControllerInterface_playerStep(self.ptr, ...)
	end
	function META:CanJump(...)
		return CLIB.btCharacterControllerInterface_canJump(self.ptr, ...)
	end
	function META:Jump(...)
		return CLIB.btCharacterControllerInterface_jump(self.ptr, ...)
	end
	function META:Reset(...)
		return CLIB.btCharacterControllerInterface_reset(self.ptr, ...)
	end
	function META:Jump2(...)
		return CLIB.btCharacterControllerInterface_jump2(self.ptr, ...)
	end
	function META:OnGround(...)
		return CLIB.btCharacterControllerInterface_onGround(self.ptr, ...)
	end
end
do -- btCollisionAlgorithmCreateFunc
	local META = {}
	library.metatables.btCollisionAlgorithmCreateFunc = META
	META.__index = function(s, k) return META[k] end
	function library.CreateCollisionAlgorithmCreateFunc(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btCollisionAlgorithmCreateFunc_new(...)
		return self
	end
	function META:CreateCollisionAlgorithm(...)
		return CLIB.btCollisionAlgorithmCreateFunc_CreateCollisionAlgorithm(self.ptr, ...)
	end
	function META:Delete(...)
		return CLIB.btCollisionAlgorithmCreateFunc_delete(self.ptr, ...)
	end
	function META:SetSwapped(...)
		return CLIB.btCollisionAlgorithmCreateFunc_setSwapped(self.ptr, ...)
	end
	function META:GetSwapped(...)
		return CLIB.btCollisionAlgorithmCreateFunc_getSwapped(self.ptr, ...)
	end
end
do -- btBroadphaseRayCallbackWrapper
	local META = {}
	library.metatables.btBroadphaseRayCallbackWrapper = META
	META.__index = function(s, k) return META[k] end
	function library.CreateBroadphaseRayCallbackWrapper(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btBroadphaseRayCallbackWrapper_new(...)
		return self
	end
end
do -- btVoronoiSimplexSolver_compute
	local META = {}
	library.metatables.btVoronoiSimplexSolver_compute = META
	META.__index = function(s, k) return META[k] end
	function META:Points(...)
		return CLIB.btVoronoiSimplexSolver_compute_points(self.ptr, ...)
	end
end
do -- btKinematicCharacterController
	local META = {}
	library.metatables.btKinematicCharacterController = META
	META.__index = function(s, k) return META[k] end
	function library.CreateKinematicCharacterController(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btKinematicCharacterController_new2(...)
		return self
	end
	function library.CreateKinematicCharacterController2(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btKinematicCharacterController_new(...)
		return self
	end
	function META:SetMaxJumpHeight(...)
		return CLIB.btKinematicCharacterController_setMaxJumpHeight(self.ptr, ...)
	end
	function META:GetMaxPenetrationDepth(...)
		return CLIB.btKinematicCharacterController_getMaxPenetrationDepth(self.ptr, ...)
	end
	function META:SetLinearVelocity(...)
		return CLIB.btKinematicCharacterController_setLinearVelocity(self.ptr, ...)
	end
	function META:SetUp(...)
		return CLIB.btKinematicCharacterController_setUp(self.ptr, ...)
	end
	function META:SetMaxPenetrationDepth(...)
		return CLIB.btKinematicCharacterController_setMaxPenetrationDepth(self.ptr, ...)
	end
	function META:GetLinearVelocity(...)
		return CLIB.btKinematicCharacterController_getLinearVelocity(self.ptr, ...)
	end
	function META:SetLinearDamping(...)
		return CLIB.btKinematicCharacterController_setLinearDamping(self.ptr, ...)
	end
	function META:SetStepHeight(...)
		return CLIB.btKinematicCharacterController_setStepHeight(self.ptr, ...)
	end
	function META:GetAngularVelocity(...)
		return CLIB.btKinematicCharacterController_getAngularVelocity(self.ptr, ...)
	end
	function META:SetUseGhostSweepTest(...)
		return CLIB.btKinematicCharacterController_setUseGhostSweepTest(self.ptr, ...)
	end
	function META:GetMaxSlope(...)
		return CLIB.btKinematicCharacterController_getMaxSlope(self.ptr, ...)
	end
	function META:GetLinearDamping(...)
		return CLIB.btKinematicCharacterController_getLinearDamping(self.ptr, ...)
	end
	function META:SetFallSpeed(...)
		return CLIB.btKinematicCharacterController_setFallSpeed(self.ptr, ...)
	end
	function META:SetGravity(...)
		return CLIB.btKinematicCharacterController_setGravity(self.ptr, ...)
	end
	function META:SetMaxSlope(...)
		return CLIB.btKinematicCharacterController_setMaxSlope(self.ptr, ...)
	end
	function META:GetGhostObject(...)
		return CLIB.btKinematicCharacterController_getGhostObject(self.ptr, ...)
	end
	function META:SetAngularDamping(...)
		return CLIB.btKinematicCharacterController_setAngularDamping(self.ptr, ...)
	end
	function META:GetGravity(...)
		return CLIB.btKinematicCharacterController_getGravity(self.ptr, ...)
	end
	function META:GetAngularDamping(...)
		return CLIB.btKinematicCharacterController_getAngularDamping(self.ptr, ...)
	end
	function META:GetFallSpeed(...)
		return CLIB.btKinematicCharacterController_getFallSpeed(self.ptr, ...)
	end
	function META:GetJumpSpeed(...)
		return CLIB.btKinematicCharacterController_getJumpSpeed(self.ptr, ...)
	end
	function META:GetStepHeight(...)
		return CLIB.btKinematicCharacterController_getStepHeight(self.ptr, ...)
	end
	function META:GetUp(...)
		return CLIB.btKinematicCharacterController_getUp(self.ptr, ...)
	end
	function META:SetAngularVelocity(...)
		return CLIB.btKinematicCharacterController_setAngularVelocity(self.ptr, ...)
	end
	function META:SetJumpSpeed(...)
		return CLIB.btKinematicCharacterController_setJumpSpeed(self.ptr, ...)
	end
	function META:ApplyImpulse(...)
		return CLIB.btKinematicCharacterController_applyImpulse(self.ptr, ...)
	end
end
do -- btTriangleShapeEx_overlap_test
	local META = {}
	library.metatables.btTriangleShapeEx_overlap_test = META
	META.__index = function(s, k) return META[k] end
	function META:Conservative(...)
		return CLIB.btTriangleShapeEx_overlap_test_conservative(self.ptr, ...)
	end
end
do -- btConvexSeparatingDistanceUtil
	local META = {}
	library.metatables.btConvexSeparatingDistanceUtil = META
	META.__index = function(s, k) return META[k] end
	function library.CreateConvexSeparatingDistanceUtil(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btConvexSeparatingDistanceUtil_new(...)
		return self
	end
	function META:UpdateSeparatingDistance(...)
		return CLIB.btConvexSeparatingDistanceUtil_updateSeparatingDistance(self.ptr, ...)
	end
	function META:InitSeparatingDistance(...)
		return CLIB.btConvexSeparatingDistanceUtil_initSeparatingDistance(self.ptr, ...)
	end
	function META:GetConservativeSeparatingDistance(...)
		return CLIB.btConvexSeparatingDistanceUtil_getConservativeSeparatingDistance(self.ptr, ...)
	end
	function META:Delete(...)
		return CLIB.btConvexSeparatingDistanceUtil_delete(self.ptr, ...)
	end
end
do -- btSoftBody_Config_setKSK_SPLT
	local META = {}
	library.metatables.btSoftBody_Config_setKSK_SPLT = META
	META.__index = function(s, k) return META[k] end
	function META:CL(...)
		return CLIB.btSoftBody_Config_setKSK_SPLT_CL(self.ptr, ...)
	end
end
do -- btSoftBody_Config_getKSK_SPLT
	local META = {}
	library.metatables.btSoftBody_Config_getKSK_SPLT = META
	META.__index = function(s, k) return META[k] end
	function META:CL(...)
		return CLIB.btSoftBody_Config_getKSK_SPLT_CL(self.ptr, ...)
	end
end
do -- btSoftBody_Config_getKSR_SPLT
	local META = {}
	library.metatables.btSoftBody_Config_getKSR_SPLT = META
	META.__index = function(s, k) return META[k] end
	function META:CL(...)
		return CLIB.btSoftBody_Config_getKSR_SPLT_CL(self.ptr, ...)
	end
end
do -- btSoftBody_Config_getKSS_SPLT
	local META = {}
	library.metatables.btSoftBody_Config_getKSS_SPLT = META
	META.__index = function(s, k) return META[k] end
	function META:CL(...)
		return CLIB.btSoftBody_Config_getKSS_SPLT_CL(self.ptr, ...)
	end
end
do -- btVoronoiSimplexSolver_backup
	local META = {}
	library.metatables.btVoronoiSimplexSolver_backup = META
	META.__index = function(s, k) return META[k] end
	function META:Closest(...)
		return CLIB.btVoronoiSimplexSolver_backup_closest(self.ptr, ...)
	end
end
do -- btSoftRigidCollisionAlgorithm
	local META = {}
	library.metatables.btSoftRigidCollisionAlgorithm = META
	META.__index = function(s, k) return META[k] end
	function library.CreateSoftRigidCollisionAlgorithm(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btSoftRigidCollisionAlgorithm_new(...)
		return self
	end
end
do -- btGeneric6DofSpringConstraint
	local META = {}
	library.metatables.btGeneric6DofSpringConstraint = META
	META.__index = function(s, k) return META[k] end
	function library.CreateGeneric6DofSpringConstraint(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btGeneric6DofSpringConstraint_new2(...)
		return self
	end
	function library.CreateGeneric6DofSpringConstraint2(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btGeneric6DofSpringConstraint_new(...)
		return self
	end
	function META:SetDamping(...)
		return CLIB.btGeneric6DofSpringConstraint_setDamping(self.ptr, ...)
	end
	function META:IsSpringEnabled(...)
		return CLIB.btGeneric6DofSpringConstraint_isSpringEnabled(self.ptr, ...)
	end
	function META:GetStiffness(...)
		return CLIB.btGeneric6DofSpringConstraint_getStiffness(self.ptr, ...)
	end
	function META:EnableSpring(...)
		return CLIB.btGeneric6DofSpringConstraint_enableSpring(self.ptr, ...)
	end
	function META:GetDamping(...)
		return CLIB.btGeneric6DofSpringConstraint_getDamping(self.ptr, ...)
	end
	function META:SetEquilibriumPoint(...)
		return CLIB.btGeneric6DofSpringConstraint_setEquilibriumPoint(self.ptr, ...)
	end
	function META:SetEquilibriumPoint2(...)
		return CLIB.btGeneric6DofSpringConstraint_setEquilibriumPoint2(self.ptr, ...)
	end
	function META:SetEquilibriumPoint3(...)
		return CLIB.btGeneric6DofSpringConstraint_setEquilibriumPoint3(self.ptr, ...)
	end
	function META:GetEquilibriumPoint(...)
		return CLIB.btGeneric6DofSpringConstraint_getEquilibriumPoint(self.ptr, ...)
	end
	function META:SetStiffness(...)
		return CLIB.btGeneric6DofSpringConstraint_setStiffness(self.ptr, ...)
	end
end
do -- btSphereBoxCollisionAlgorithm
	local META = {}
	library.metatables.btSphereBoxCollisionAlgorithm = META
	META.__index = function(s, k) return META[k] end
	function library.CreateSphereBoxCollisionAlgorithm(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btSphereBoxCollisionAlgorithm_new(...)
		return self
	end
	function META:GetSpherePenetration(...)
		return CLIB.btSphereBoxCollisionAlgorithm_getSpherePenetration(self.ptr, ...)
	end
	function META:GetSphereDistance(...)
		return CLIB.btSphereBoxCollisionAlgorithm_getSphereDistance(self.ptr, ...)
	end
end
do -- btSoftBody_Config_setKSS_SPLT
	local META = {}
	library.metatables.btSoftBody_Config_setKSS_SPLT = META
	META.__index = function(s, k) return META[k] end
	function META:CL(...)
		return CLIB.btSoftBody_Config_setKSS_SPLT_CL(self.ptr, ...)
	end
end
do -- btSoftBody_Config_setKSR_SPLT
	local META = {}
	library.metatables.btSoftBody_Config_setKSR_SPLT = META
	META.__index = function(s, k) return META[k] end
	function META:CL(...)
		return CLIB.btSoftBody_Config_setKSR_SPLT_CL(self.ptr, ...)
	end
end
do -- btSoftBody_ImplicitFnWrapper
	local META = {}
	library.metatables.btSoftBody_ImplicitFnWrapper = META
	META.__index = function(s, k) return META[k] end
	function library.CreateSoftBody_ImplicitFnWrapper(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btSoftBody_ImplicitFnWrapper_new(...)
		return self
	end
end
do -- btSoftSoftCollisionAlgorithm
	local META = {}
	library.metatables.btSoftSoftCollisionAlgorithm = META
	META.__index = function(s, k) return META[k] end
	function library.CreateSoftSoftCollisionAlgorithm(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btSoftSoftCollisionAlgorithm_new2(...)
		return self
	end
	function library.CreateSoftSoftCollisionAlgorithm2(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btSoftSoftCollisionAlgorithm_new(...)
		return self
	end
end
do -- btScaledBvhTriangleMeshShape
	local META = {}
	library.metatables.btScaledBvhTriangleMeshShape = META
	META.__index = function(s, k) return META[k] end
	function library.CreateScaledBvhTriangleMeshShape(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btScaledBvhTriangleMeshShape_new(...)
		return self
	end
	function META:GetChildShape(...)
		return CLIB.btScaledBvhTriangleMeshShape_getChildShape(self.ptr, ...)
	end
end
do -- btCompoundCollisionAlgorithm
	local META = {}
	library.metatables.btCompoundCollisionAlgorithm = META
	META.__index = function(s, k) return META[k] end
	function library.CreateCompoundCollisionAlgorithm(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btCompoundCollisionAlgorithm_new(...)
		return self
	end
	function META:GetChildAlgorithm(...)
		return CLIB.btCompoundCollisionAlgorithm_getChildAlgorithm(self.ptr, ...)
	end
end
do -- btSoftBodyWorldInfo_setWater
	local META = {}
	library.metatables.btSoftBodyWorldInfo_setWater = META
	META.__index = function(s, k) return META[k] end
	function META:Offset(...)
		return CLIB.btSoftBodyWorldInfo_setWater_offset(self.ptr, ...)
	end
	function META:Normal(...)
		return CLIB.btSoftBodyWorldInfo_setWater_normal(self.ptr, ...)
	end
	function META:Density(...)
		return CLIB.btSoftBodyWorldInfo_setWater_density(self.ptr, ...)
	end
end
do -- btPrimitiveTriangle_get_edge
	local META = {}
	library.metatables.btPrimitiveTriangle_get_edge = META
	META.__index = function(s, k) return META[k] end
	function META:Plane(...)
		return CLIB.btPrimitiveTriangle_get_edge_plane(self.ptr, ...)
	end
end
do -- btHashedOverlappingPairCache
	local META = {}
	library.metatables.btHashedOverlappingPairCache = META
	META.__index = function(s, k) return META[k] end
	function library.CreateHashedOverlappingPairCache(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btHashedOverlappingPairCache_new(...)
		return self
	end
	function META:NeedsBroadphaseCollision(...)
		return CLIB.btHashedOverlappingPairCache_needsBroadphaseCollision(self.ptr, ...)
	end
	function META:GetCount(...)
		return CLIB.btHashedOverlappingPairCache_GetCount(self.ptr, ...)
	end
	function META:GetOverlapFilterCallback(...)
		return CLIB.btHashedOverlappingPairCache_getOverlapFilterCallback(self.ptr, ...)
	end
end
do -- btSoftBodyWorldInfo_getWater
	local META = {}
	library.metatables.btSoftBodyWorldInfo_getWater = META
	META.__index = function(s, k) return META[k] end
	function META:Offset(...)
		return CLIB.btSoftBodyWorldInfo_getWater_offset(self.ptr, ...)
	end
	function META:Density(...)
		return CLIB.btSoftBodyWorldInfo_getWater_density(self.ptr, ...)
	end
	function META:Normal(...)
		return CLIB.btSoftBodyWorldInfo_getWater_normal(self.ptr, ...)
	end
end
do -- btSortedOverlappingPairCache
	local META = {}
	library.metatables.btSortedOverlappingPairCache = META
	META.__index = function(s, k) return META[k] end
	function library.CreateSortedOverlappingPairCache(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btSortedOverlappingPairCache_new(...)
		return self
	end
	function META:GetOverlapFilterCallback(...)
		return CLIB.btSortedOverlappingPairCache_getOverlapFilterCallback(self.ptr, ...)
	end
	function META:NeedsBroadphaseCollision(...)
		return CLIB.btSortedOverlappingPairCache_needsBroadphaseCollision(self.ptr, ...)
	end
end
do -- btDbvtBroadphase_setUpdates
	local META = {}
	library.metatables.btDbvtBroadphase_setUpdates = META
	META.__index = function(s, k) return META[k] end
	function META:Ratio(...)
		return CLIB.btDbvtBroadphase_setUpdates_ratio(self.ptr, ...)
	end
	function META:Done(...)
		return CLIB.btDbvtBroadphase_setUpdates_done(self.ptr, ...)
	end
	function META:Call(...)
		return CLIB.btDbvtBroadphase_setUpdates_call(self.ptr, ...)
	end
end
do -- btMultiBodyConstraintSolver
	local META = {}
	library.metatables.btMultiBodyConstraintSolver = META
	META.__index = function(s, k) return META[k] end
	function library.CreateMultiBodyConstraintSolver(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btMultiBodyConstraintSolver_new(...)
		return self
	end
	function META:SolveMultiBodyGroup(...)
		return CLIB.btMultiBodyConstraintSolver_solveMultiBodyGroup(self.ptr, ...)
	end
	function META:SolveGroupCacheFriendlyFinish(...)
		return CLIB.btMultiBodyConstraintSolver_solveGroupCacheFriendlyFinish(self.ptr, ...)
	end
end
do -- btMultiBodySolverConstraint
	local META = {}
	library.metatables.btMultiBodySolverConstraint = META
	META.__index = function(s, k) return META[k] end
	function library.CreateMultiBodySolverConstraint(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btMultiBodySolverConstraint_new(...)
		return self
	end
	function META:GetJacAindex(...)
		return CLIB.btMultiBodySolverConstraint_getJacAindex(self.ptr, ...)
	end
	function META:GetSolverBodyIdA(...)
		return CLIB.btMultiBodySolverConstraint_getSolverBodyIdA(self.ptr, ...)
	end
	function META:SetJacAindex(...)
		return CLIB.btMultiBodySolverConstraint_setJacAindex(self.ptr, ...)
	end
	function META:GetDeltaVelBindex(...)
		return CLIB.btMultiBodySolverConstraint_getDeltaVelBindex(self.ptr, ...)
	end
	function META:SetAppliedPushImpulse(...)
		return CLIB.btMultiBodySolverConstraint_setAppliedPushImpulse(self.ptr, ...)
	end
	function META:GetRelpos2CrossNormal(...)
		return CLIB.btMultiBodySolverConstraint_getRelpos2CrossNormal(self.ptr, ...)
	end
	function META:GetAngularComponentA(...)
		return CLIB.btMultiBodySolverConstraint_getAngularComponentA(self.ptr, ...)
	end
	function META:SetDeltaVelAindex(...)
		return CLIB.btMultiBodySolverConstraint_setDeltaVelAindex(self.ptr, ...)
	end
	function META:GetContactNormal2(...)
		return CLIB.btMultiBodySolverConstraint_getContactNormal2(self.ptr, ...)
	end
	function META:SetCfm(...)
		return CLIB.btMultiBodySolverConstraint_setCfm(self.ptr, ...)
	end
	function META:GetAppliedImpulse(...)
		return CLIB.btMultiBodySolverConstraint_getAppliedImpulse(self.ptr, ...)
	end
	function META:SetLinkA(...)
		return CLIB.btMultiBodySolverConstraint_setLinkA(self.ptr, ...)
	end
	function META:SetOrgDofIndex(...)
		return CLIB.btMultiBodySolverConstraint_setOrgDofIndex(self.ptr, ...)
	end
	function META:SetLowerLimit(...)
		return CLIB.btMultiBodySolverConstraint_setLowerLimit(self.ptr, ...)
	end
	function META:GetAppliedPushImpulse(...)
		return CLIB.btMultiBodySolverConstraint_getAppliedPushImpulse(self.ptr, ...)
	end
	function META:GetOrgDofIndex(...)
		return CLIB.btMultiBodySolverConstraint_getOrgDofIndex(self.ptr, ...)
	end
	function META:GetMultiBodyA(...)
		return CLIB.btMultiBodySolverConstraint_getMultiBodyA(self.ptr, ...)
	end
	function META:GetLowerLimit(...)
		return CLIB.btMultiBodySolverConstraint_getLowerLimit(self.ptr, ...)
	end
	function META:GetOriginalContactPoint(...)
		return CLIB.btMultiBodySolverConstraint_getOriginalContactPoint(self.ptr, ...)
	end
	function META:GetRhsPenetration(...)
		return CLIB.btMultiBodySolverConstraint_getRhsPenetration(self.ptr, ...)
	end
	function META:SetFrictionIndex(...)
		return CLIB.btMultiBodySolverConstraint_setFrictionIndex(self.ptr, ...)
	end
	function META:GetJacDiagABInv(...)
		return CLIB.btMultiBodySolverConstraint_getJacDiagABInv(self.ptr, ...)
	end
	function META:GetMultiBodyB(...)
		return CLIB.btMultiBodySolverConstraint_getMultiBodyB(self.ptr, ...)
	end
	function META:GetFriction(...)
		return CLIB.btMultiBodySolverConstraint_getFriction(self.ptr, ...)
	end
	function META:SetOverrideNumSolverIterations(...)
		return CLIB.btMultiBodySolverConstraint_setOverrideNumSolverIterations(self.ptr, ...)
	end
	function META:GetCfm(...)
		return CLIB.btMultiBodySolverConstraint_getCfm(self.ptr, ...)
	end
	function META:SetJacBindex(...)
		return CLIB.btMultiBodySolverConstraint_setJacBindex(self.ptr, ...)
	end
	function META:Delete(...)
		return CLIB.btMultiBodySolverConstraint_delete(self.ptr, ...)
	end
	function META:SetUpperLimit(...)
		return CLIB.btMultiBodySolverConstraint_setUpperLimit(self.ptr, ...)
	end
	function META:SetUnusedPadding4(...)
		return CLIB.btMultiBodySolverConstraint_setUnusedPadding4(self.ptr, ...)
	end
	function META:SetSolverBodyIdB(...)
		return CLIB.btMultiBodySolverConstraint_setSolverBodyIdB(self.ptr, ...)
	end
	function META:SetSolverBodyIdA(...)
		return CLIB.btMultiBodySolverConstraint_setSolverBodyIdA(self.ptr, ...)
	end
	function META:SetRhsPenetration(...)
		return CLIB.btMultiBodySolverConstraint_setRhsPenetration(self.ptr, ...)
	end
	function META:SetRhs(...)
		return CLIB.btMultiBodySolverConstraint_setRhs(self.ptr, ...)
	end
	function META:SetRelpos2CrossNormal(...)
		return CLIB.btMultiBodySolverConstraint_setRelpos2CrossNormal(self.ptr, ...)
	end
	function META:SetRelpos1CrossNormal(...)
		return CLIB.btMultiBodySolverConstraint_setRelpos1CrossNormal(self.ptr, ...)
	end
	function META:SetOriginalContactPoint(...)
		return CLIB.btMultiBodySolverConstraint_setOriginalContactPoint(self.ptr, ...)
	end
	function META:SetOrgConstraint(...)
		return CLIB.btMultiBodySolverConstraint_setOrgConstraint(self.ptr, ...)
	end
	function META:SetMultiBodyB(...)
		return CLIB.btMultiBodySolverConstraint_setMultiBodyB(self.ptr, ...)
	end
	function META:SetMultiBodyA(...)
		return CLIB.btMultiBodySolverConstraint_setMultiBodyA(self.ptr, ...)
	end
	function META:SetLinkB(...)
		return CLIB.btMultiBodySolverConstraint_setLinkB(self.ptr, ...)
	end
	function META:SetJacDiagABInv(...)
		return CLIB.btMultiBodySolverConstraint_setJacDiagABInv(self.ptr, ...)
	end
	function META:SetFriction(...)
		return CLIB.btMultiBodySolverConstraint_setFriction(self.ptr, ...)
	end
	function META:SetDeltaVelBindex(...)
		return CLIB.btMultiBodySolverConstraint_setDeltaVelBindex(self.ptr, ...)
	end
	function META:SetContactNormal2(...)
		return CLIB.btMultiBodySolverConstraint_setContactNormal2(self.ptr, ...)
	end
	function META:SetContactNormal1(...)
		return CLIB.btMultiBodySolverConstraint_setContactNormal1(self.ptr, ...)
	end
	function META:SetAppliedImpulse(...)
		return CLIB.btMultiBodySolverConstraint_setAppliedImpulse(self.ptr, ...)
	end
	function META:SetAngularComponentB(...)
		return CLIB.btMultiBodySolverConstraint_setAngularComponentB(self.ptr, ...)
	end
	function META:SetAngularComponentA(...)
		return CLIB.btMultiBodySolverConstraint_setAngularComponentA(self.ptr, ...)
	end
	function META:GetUpperLimit(...)
		return CLIB.btMultiBodySolverConstraint_getUpperLimit(self.ptr, ...)
	end
	function META:GetUnusedPadding4(...)
		return CLIB.btMultiBodySolverConstraint_getUnusedPadding4(self.ptr, ...)
	end
	function META:GetRhs(...)
		return CLIB.btMultiBodySolverConstraint_getRhs(self.ptr, ...)
	end
	function META:GetLinkB(...)
		return CLIB.btMultiBodySolverConstraint_getLinkB(self.ptr, ...)
	end
	function META:GetLinkA(...)
		return CLIB.btMultiBodySolverConstraint_getLinkA(self.ptr, ...)
	end
	function META:GetJacBindex(...)
		return CLIB.btMultiBodySolverConstraint_getJacBindex(self.ptr, ...)
	end
	function META:GetFrictionIndex(...)
		return CLIB.btMultiBodySolverConstraint_getFrictionIndex(self.ptr, ...)
	end
	function META:GetContactNormal1(...)
		return CLIB.btMultiBodySolverConstraint_getContactNormal1(self.ptr, ...)
	end
	function META:GetAngularComponentB(...)
		return CLIB.btMultiBodySolverConstraint_getAngularComponentB(self.ptr, ...)
	end
	function META:GetOverrideNumSolverIterations(...)
		return CLIB.btMultiBodySolverConstraint_getOverrideNumSolverIterations(self.ptr, ...)
	end
	function META:GetOrgConstraint(...)
		return CLIB.btMultiBodySolverConstraint_getOrgConstraint(self.ptr, ...)
	end
	function META:GetDeltaVelAindex(...)
		return CLIB.btMultiBodySolverConstraint_getDeltaVelAindex(self.ptr, ...)
	end
	function META:GetSolverBodyIdB(...)
		return CLIB.btMultiBodySolverConstraint_getSolverBodyIdB(self.ptr, ...)
	end
	function META:GetRelpos1CrossNormal(...)
		return CLIB.btMultiBodySolverConstraint_getRelpos1CrossNormal(self.ptr, ...)
	end
end
do -- btContinuousConvexCollision
	local META = {}
	library.metatables.btContinuousConvexCollision = META
	META.__index = function(s, k) return META[k] end
	function library.CreateContinuousConvexCollision(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btContinuousConvexCollision_new(...)
		return self
	end
	function library.CreateContinuousConvexCollision2(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btContinuousConvexCollision_new2(...)
		return self
	end
end
do -- btDbvtBroadphase_getUpdates
	local META = {}
	library.metatables.btDbvtBroadphase_getUpdates = META
	META.__index = function(s, k) return META[k] end
	function META:Done(...)
		return CLIB.btDbvtBroadphase_getUpdates_done(self.ptr, ...)
	end
	function META:Call(...)
		return CLIB.btDbvtBroadphase_getUpdates_call(self.ptr, ...)
	end
	function META:Ratio(...)
		return CLIB.btDbvtBroadphase_getUpdates_ratio(self.ptr, ...)
	end
end
do -- btSoftBody_Impulse_operator
	local META = {}
	library.metatables.btSoftBody_Impulse_operator = META
	META.__index = function(s, k) return META[k] end
	function META:M(...)
		return CLIB.btSoftBody_Impulse_operator_m(self.ptr, ...)
	end
	function META:N(...)
		return CLIB.btSoftBody_Impulse_operator_n(self.ptr, ...)
	end
end
do -- btAABB_appy_transform_trans
	local META = {}
	library.metatables.btAABB_appy_transform_trans = META
	META.__index = function(s, k) return META[k] end
	function META:Cache(...)
		return CLIB.btAABB_appy_transform_trans_cache(self.ptr, ...)
	end
end
do -- btQuantizedBvhTree_get_node
	local META = {}
	library.metatables.btQuantizedBvhTree_get_node = META
	META.__index = function(s, k) return META[k] end
	function META:Pointer(...)
		return CLIB.btQuantizedBvhTree_get_node_pointer(self.ptr, ...)
	end
end
do -- btMultiBodySliderConstraint
	local META = {}
	library.metatables.btMultiBodySliderConstraint = META
	META.__index = function(s, k) return META[k] end
	function library.CreateMultiBodySliderConstraint(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btMultiBodySliderConstraint_new(...)
		return self
	end
	function library.CreateMultiBodySliderConstraint2(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btMultiBodySliderConstraint_new2(...)
		return self
	end
	function META:GetFrameInB(...)
		return CLIB.btMultiBodySliderConstraint_getFrameInB(self.ptr, ...)
	end
	function META:GetFrameInA(...)
		return CLIB.btMultiBodySliderConstraint_getFrameInA(self.ptr, ...)
	end
	function META:SetPivotInB(...)
		return CLIB.btMultiBodySliderConstraint_setPivotInB(self.ptr, ...)
	end
	function META:SetPivotInA(...)
		return CLIB.btMultiBodySliderConstraint_setPivotInA(self.ptr, ...)
	end
	function META:SetJointAxis(...)
		return CLIB.btMultiBodySliderConstraint_setJointAxis(self.ptr, ...)
	end
	function META:SetFrameInB(...)
		return CLIB.btMultiBodySliderConstraint_setFrameInB(self.ptr, ...)
	end
	function META:SetFrameInA(...)
		return CLIB.btMultiBodySliderConstraint_setFrameInA(self.ptr, ...)
	end
	function META:GetPivotInB(...)
		return CLIB.btMultiBodySliderConstraint_getPivotInB(self.ptr, ...)
	end
	function META:GetPivotInA(...)
		return CLIB.btMultiBodySliderConstraint_getPivotInA(self.ptr, ...)
	end
	function META:GetJointAxis(...)
		return CLIB.btMultiBodySliderConstraint_getJointAxis(self.ptr, ...)
	end
end
do -- btGImpactCollisionAlgorithm
	local META = {}
	library.metatables.btGImpactCollisionAlgorithm = META
	META.__index = function(s, k) return META[k] end
	function library.CreateGImpactCollisionAlgorithm(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btGImpactCollisionAlgorithm_new(...)
		return self
	end
	function META:InternalGetResultOut(...)
		return CLIB.btGImpactCollisionAlgorithm_internalGetResultOut(self.ptr, ...)
	end
	function META:GetFace1(...)
		return CLIB.btGImpactCollisionAlgorithm_getFace1(self.ptr, ...)
	end
	function META:SetPart1(...)
		return CLIB.btGImpactCollisionAlgorithm_setPart1(self.ptr, ...)
	end
	function META:GetPart0(...)
		return CLIB.btGImpactCollisionAlgorithm_getPart0(self.ptr, ...)
	end
	function META:SetFace0(...)
		return CLIB.btGImpactCollisionAlgorithm_setFace0(self.ptr, ...)
	end
	function META:RegisterAlgorithm(...)
		return CLIB.btGImpactCollisionAlgorithm_registerAlgorithm(self.ptr, ...)
	end
	function META:SetFace1(...)
		return CLIB.btGImpactCollisionAlgorithm_setFace1(self.ptr, ...)
	end
	function META:GetPart1(...)
		return CLIB.btGImpactCollisionAlgorithm_getPart1(self.ptr, ...)
	end
	function META:SetPart0(...)
		return CLIB.btGImpactCollisionAlgorithm_setPart0(self.ptr, ...)
	end
	function META:GetFace0(...)
		return CLIB.btGImpactCollisionAlgorithm_getFace0(self.ptr, ...)
	end
end
do -- btConvex2dConvex2dAlgorithm
	local META = {}
	library.metatables.btConvex2dConvex2dAlgorithm = META
	META.__index = function(s, k) return META[k] end
	function library.CreateConvex2dConvex2dAlgorithm(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btConvex2dConvex2dAlgorithm_new(...)
		return self
	end
	function META:SetLowLevelOfDetail(...)
		return CLIB.btConvex2dConvex2dAlgorithm_setLowLevelOfDetail(self.ptr, ...)
	end
	function META:GetManifold(...)
		return CLIB.btConvex2dConvex2dAlgorithm_getManifold(self.ptr, ...)
	end
end
do -- btEmptyAlgorithm_CreateFunc
	local META = {}
	library.metatables.btEmptyAlgorithm_CreateFunc = META
	META.__index = function(s, k) return META[k] end
	function library.CreateEmptyAlgorithm_CreateFunc(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btEmptyAlgorithm_CreateFunc_new(...)
		return self
	end
end
do -- btWheelInfoConstructionInfo
	local META = {}
	library.metatables.btWheelInfoConstructionInfo = META
	META.__index = function(s, k) return META[k] end
	function library.CreateWheelInfoConstructionInfo(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btWheelInfoConstructionInfo_new(...)
		return self
	end
	function META:SetWheelsDampingRelaxation(...)
		return CLIB.btWheelInfoConstructionInfo_setWheelsDampingRelaxation(self.ptr, ...)
	end
	function META:SetWheelRadius(...)
		return CLIB.btWheelInfoConstructionInfo_setWheelRadius(self.ptr, ...)
	end
	function META:SetWheelDirectionCS(...)
		return CLIB.btWheelInfoConstructionInfo_setWheelDirectionCS(self.ptr, ...)
	end
	function META:SetWheelAxleCS(...)
		return CLIB.btWheelInfoConstructionInfo_setWheelAxleCS(self.ptr, ...)
	end
	function META:SetSuspensionStiffness(...)
		return CLIB.btWheelInfoConstructionInfo_setSuspensionStiffness(self.ptr, ...)
	end
	function META:SetSuspensionRestLength(...)
		return CLIB.btWheelInfoConstructionInfo_setSuspensionRestLength(self.ptr, ...)
	end
	function META:SetMaxSuspensionTravelCm(...)
		return CLIB.btWheelInfoConstructionInfo_setMaxSuspensionTravelCm(self.ptr, ...)
	end
	function META:SetMaxSuspensionForce(...)
		return CLIB.btWheelInfoConstructionInfo_setMaxSuspensionForce(self.ptr, ...)
	end
	function META:SetFrictionSlip(...)
		return CLIB.btWheelInfoConstructionInfo_setFrictionSlip(self.ptr, ...)
	end
	function META:SetChassisConnectionCS(...)
		return CLIB.btWheelInfoConstructionInfo_setChassisConnectionCS(self.ptr, ...)
	end
	function META:SetBIsFrontWheel(...)
		return CLIB.btWheelInfoConstructionInfo_setBIsFrontWheel(self.ptr, ...)
	end
	function META:GetWheelsDampingRelaxation(...)
		return CLIB.btWheelInfoConstructionInfo_getWheelsDampingRelaxation(self.ptr, ...)
	end
	function META:GetWheelRadius(...)
		return CLIB.btWheelInfoConstructionInfo_getWheelRadius(self.ptr, ...)
	end
	function META:GetWheelAxleCS(...)
		return CLIB.btWheelInfoConstructionInfo_getWheelAxleCS(self.ptr, ...)
	end
	function META:GetSuspensionStiffness(...)
		return CLIB.btWheelInfoConstructionInfo_getSuspensionStiffness(self.ptr, ...)
	end
	function META:GetSuspensionRestLength(...)
		return CLIB.btWheelInfoConstructionInfo_getSuspensionRestLength(self.ptr, ...)
	end
	function META:GetMaxSuspensionForce(...)
		return CLIB.btWheelInfoConstructionInfo_getMaxSuspensionForce(self.ptr, ...)
	end
	function META:GetFrictionSlip(...)
		return CLIB.btWheelInfoConstructionInfo_getFrictionSlip(self.ptr, ...)
	end
	function META:GetChassisConnectionCS(...)
		return CLIB.btWheelInfoConstructionInfo_getChassisConnectionCS(self.ptr, ...)
	end
	function META:GetBIsFrontWheel(...)
		return CLIB.btWheelInfoConstructionInfo_getBIsFrontWheel(self.ptr, ...)
	end
	function META:GetMaxSuspensionTravelCm(...)
		return CLIB.btWheelInfoConstructionInfo_getMaxSuspensionTravelCm(self.ptr, ...)
	end
	function META:SetWheelsDampingCompression(...)
		return CLIB.btWheelInfoConstructionInfo_setWheelsDampingCompression(self.ptr, ...)
	end
	function META:GetWheelDirectionCS(...)
		return CLIB.btWheelInfoConstructionInfo_getWheelDirectionCS(self.ptr, ...)
	end
	function META:Delete(...)
		return CLIB.btWheelInfoConstructionInfo_delete(self.ptr, ...)
	end
	function META:GetWheelsDampingCompression(...)
		return CLIB.btWheelInfoConstructionInfo_getWheelsDampingCompression(self.ptr, ...)
	end
end
do -- btCompoundShapeChild_array
	local META = {}
	library.metatables.btCompoundShapeChild_array = META
	META.__index = function(s, k) return META[k] end
	function META:At(...)
		return CLIB.btCompoundShapeChild_array_at(self.ptr, ...)
	end
end
do -- btSoftBody_Config_setKSKHR
	local META = {}
	library.metatables.btSoftBody_Config_setKSKHR = META
	META.__index = function(s, k) return META[k] end
	function META:CL(...)
		return CLIB.btSoftBody_Config_setKSKHR_CL(self.ptr, ...)
	end
end
do -- btSoftBody_Config_getKSRHR
	local META = {}
	library.metatables.btSoftBody_Config_getKSRHR = META
	META.__index = function(s, k) return META[k] end
	function META:CL(...)
		return CLIB.btSoftBody_Config_getKSRHR_CL(self.ptr, ...)
	end
end
do -- btSoftBody_RayFromToCaster
	local META = {}
	library.metatables.btSoftBody_RayFromToCaster = META
	META.__index = function(s, k) return META[k] end
	function library.CreateSoftBody_RayFromToCaster(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btSoftBody_RayFromToCaster_new(...)
		return self
	end
	function META:GetTests(...)
		return CLIB.btSoftBody_RayFromToCaster_getTests(self.ptr, ...)
	end
	function META:GetFace(...)
		return CLIB.btSoftBody_RayFromToCaster_getFace(self.ptr, ...)
	end
	function META:SetRayTo(...)
		return CLIB.btSoftBody_RayFromToCaster_setRayTo(self.ptr, ...)
	end
	function META:RayFromToTriangle(...)
		return CLIB.btSoftBody_RayFromToCaster_rayFromToTriangle(self.ptr, ...)
	end
	function META:GetMint(...)
		return CLIB.btSoftBody_RayFromToCaster_getMint(self.ptr, ...)
	end
	function META:SetRayFrom(...)
		return CLIB.btSoftBody_RayFromToCaster_setRayFrom(self.ptr, ...)
	end
	function META:SetTests(...)
		return CLIB.btSoftBody_RayFromToCaster_setTests(self.ptr, ...)
	end
	function META:SetMint(...)
		return CLIB.btSoftBody_RayFromToCaster_setMint(self.ptr, ...)
	end
	function META:SetFace(...)
		return CLIB.btSoftBody_RayFromToCaster_setFace(self.ptr, ...)
	end
	function META:RayFromToTriangle2(...)
		return CLIB.btSoftBody_RayFromToCaster_rayFromToTriangle2(self.ptr, ...)
	end
	function META:GetRayTo(...)
		return CLIB.btSoftBody_RayFromToCaster_getRayTo(self.ptr, ...)
	end
	function META:SetRayNormalizedDirection(...)
		return CLIB.btSoftBody_RayFromToCaster_setRayNormalizedDirection(self.ptr, ...)
	end
	function META:GetRayFrom(...)
		return CLIB.btSoftBody_RayFromToCaster_getRayFrom(self.ptr, ...)
	end
	function META:GetRayNormalizedDirection(...)
		return CLIB.btSoftBody_RayFromToCaster_getRayNormalizedDirection(self.ptr, ...)
	end
end
do -- btGImpactQuantizedBvh_find
	local META = {}
	library.metatables.btGImpactQuantizedBvh_find = META
	META.__index = function(s, k) return META[k] end
	function META:Collision(...)
		return CLIB.btGImpactQuantizedBvh_find_collision(self.ptr, ...)
	end
end
do -- btBoxBoxCollisionAlgorithm
	local META = {}
	library.metatables.btBoxBoxCollisionAlgorithm = META
	META.__index = function(s, k) return META[k] end
	function library.CreateBoxBoxCollisionAlgorithm(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btBoxBoxCollisionAlgorithm_new2(...)
		return self
	end
	function library.CreateBoxBoxCollisionAlgorithm2(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btBoxBoxCollisionAlgorithm_new(...)
		return self
	end
end
do -- btSoftBody_Config_setKSSHR
	local META = {}
	library.metatables.btSoftBody_Config_setKSSHR = META
	META.__index = function(s, k) return META[k] end
	function META:CL(...)
		return CLIB.btSoftBody_Config_setKSSHR_CL(self.ptr, ...)
	end
end
do -- btSoftBody_Config_setKSRHR
	local META = {}
	library.metatables.btSoftBody_Config_setKSRHR = META
	META.__index = function(s, k) return META[k] end
	function META:CL(...)
		return CLIB.btSoftBody_Config_setKSRHR_CL(self.ptr, ...)
	end
end
do -- btTriangleIndexVertexArray
	local META = {}
	library.metatables.btTriangleIndexVertexArray = META
	META.__index = function(s, k) return META[k] end
	function library.CreateTriangleIndexVertexArray(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btTriangleIndexVertexArray_new(...)
		return self
	end
	function library.CreateTriangleIndexVertexArray2(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btTriangleIndexVertexArray_new2(...)
		return self
	end
	function META:GetIndexedMeshArray(...)
		return CLIB.btTriangleIndexVertexArray_getIndexedMeshArray(self.ptr, ...)
	end
	function META:AddIndexedMesh(...)
		return CLIB.btTriangleIndexVertexArray_addIndexedMesh(self.ptr, ...)
	end
end
do -- btSoftBody_Config_getKSKHR
	local META = {}
	library.metatables.btSoftBody_Config_getKSKHR = META
	META.__index = function(s, k) return META[k] end
	function META:CL(...)
		return CLIB.btSoftBody_Config_getKSKHR_CL(self.ptr, ...)
	end
end
do -- btMultiBodyFixedConstraint
	local META = {}
	library.metatables.btMultiBodyFixedConstraint = META
	META.__index = function(s, k) return META[k] end
	function library.CreateMultiBodyFixedConstraint(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btMultiBodyFixedConstraint_new(...)
		return self
	end
	function library.CreateMultiBodyFixedConstraint2(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btMultiBodyFixedConstraint_new2(...)
		return self
	end
	function META:GetPivotInB(...)
		return CLIB.btMultiBodyFixedConstraint_getPivotInB(self.ptr, ...)
	end
	function META:SetPivotInA(...)
		return CLIB.btMultiBodyFixedConstraint_setPivotInA(self.ptr, ...)
	end
	function META:SetFrameInB(...)
		return CLIB.btMultiBodyFixedConstraint_setFrameInB(self.ptr, ...)
	end
	function META:GetFrameInA(...)
		return CLIB.btMultiBodyFixedConstraint_getFrameInA(self.ptr, ...)
	end
	function META:SetFrameInA(...)
		return CLIB.btMultiBodyFixedConstraint_setFrameInA(self.ptr, ...)
	end
	function META:GetPivotInA(...)
		return CLIB.btMultiBodyFixedConstraint_getPivotInA(self.ptr, ...)
	end
	function META:SetPivotInB(...)
		return CLIB.btMultiBodyFixedConstraint_setPivotInB(self.ptr, ...)
	end
	function META:GetFrameInB(...)
		return CLIB.btMultiBodyFixedConstraint_getFrameInB(self.ptr, ...)
	end
end
do -- btSoftBody_Config_getKSSHR
	local META = {}
	library.metatables.btSoftBody_Config_getKSSHR = META
	META.__index = function(s, k) return META[k] end
	function META:CL(...)
		return CLIB.btSoftBody_Config_getKSSHR_CL(self.ptr, ...)
	end
end
do -- btSoftBodyWorldInfo_getAir
	local META = {}
	library.metatables.btSoftBodyWorldInfo_getAir = META
	META.__index = function(s, k) return META[k] end
	function META:Density(...)
		return CLIB.btSoftBodyWorldInfo_getAir_density(self.ptr, ...)
	end
end
do -- btSoftBody_AJoint_IControl
	local META = {}
	library.metatables.btSoftBody_AJoint_IControl = META
	META.__index = function(s, k) return META[k] end
	function library.CreateSoftBody_AJoint_IControl(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btSoftBody_AJoint_IControl_new(...)
		return self
	end
	function META:Speed(...)
		return CLIB.btSoftBody_AJoint_IControl_Speed(self.ptr, ...)
	end
	function META:Delete(...)
		return CLIB.btSoftBody_AJoint_IControl_delete(self.ptr, ...)
	end
	function META:Default(...)
		return CLIB.btSoftBody_AJoint_IControl_Default(self.ptr, ...)
	end
	function META:Prepare(...)
		return CLIB.btSoftBody_AJoint_IControl_Prepare(self.ptr, ...)
	end
end
do -- btTranslationalLimitMotor2
	local META = {}
	library.metatables.btTranslationalLimitMotor2 = META
	META.__index = function(s, k) return META[k] end
	function library.CreateTranslationalLimitMotor2(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btTranslationalLimitMotor2_new2(...)
		return self
	end
	function library.CreateTranslationalLimitMotor22(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btTranslationalLimitMotor2_new(...)
		return self
	end
	function META:GetSpringStiffness(...)
		return CLIB.btTranslationalLimitMotor2_getSpringStiffness(self.ptr, ...)
	end
	function META:GetEquilibriumPoint(...)
		return CLIB.btTranslationalLimitMotor2_getEquilibriumPoint(self.ptr, ...)
	end
	function META:Delete(...)
		return CLIB.btTranslationalLimitMotor2_delete(self.ptr, ...)
	end
	function META:GetTargetVelocity(...)
		return CLIB.btTranslationalLimitMotor2_getTargetVelocity(self.ptr, ...)
	end
	function META:GetSpringDamping(...)
		return CLIB.btTranslationalLimitMotor2_getSpringDamping(self.ptr, ...)
	end
	function META:SetServoTarget(...)
		return CLIB.btTranslationalLimitMotor2_setServoTarget(self.ptr, ...)
	end
	function META:GetSpringDampingLimited(...)
		return CLIB.btTranslationalLimitMotor2_getSpringDampingLimited(self.ptr, ...)
	end
	function META:SetCurrentLimitErrorHi(...)
		return CLIB.btTranslationalLimitMotor2_setCurrentLimitErrorHi(self.ptr, ...)
	end
	function META:SetCurrentLimitError(...)
		return CLIB.btTranslationalLimitMotor2_setCurrentLimitError(self.ptr, ...)
	end
	function META:GetEnableSpring(...)
		return CLIB.btTranslationalLimitMotor2_getEnableSpring(self.ptr, ...)
	end
	function META:SetStopERP(...)
		return CLIB.btTranslationalLimitMotor2_setStopERP(self.ptr, ...)
	end
	function META:GetEnableMotor(...)
		return CLIB.btTranslationalLimitMotor2_getEnableMotor(self.ptr, ...)
	end
	function META:SetBounce(...)
		return CLIB.btTranslationalLimitMotor2_setBounce(self.ptr, ...)
	end
	function META:GetMotorERP(...)
		return CLIB.btTranslationalLimitMotor2_getMotorERP(self.ptr, ...)
	end
	function META:GetLowerLimit(...)
		return CLIB.btTranslationalLimitMotor2_getLowerLimit(self.ptr, ...)
	end
	function META:GetMaxMotorForce(...)
		return CLIB.btTranslationalLimitMotor2_getMaxMotorForce(self.ptr, ...)
	end
	function META:GetCurrentLimit(...)
		return CLIB.btTranslationalLimitMotor2_getCurrentLimit(self.ptr, ...)
	end
	function META:GetServoMotor(...)
		return CLIB.btTranslationalLimitMotor2_getServoMotor(self.ptr, ...)
	end
	function META:TestLimitValue(...)
		return CLIB.btTranslationalLimitMotor2_testLimitValue(self.ptr, ...)
	end
	function META:GetMotorCFM(...)
		return CLIB.btTranslationalLimitMotor2_getMotorCFM(self.ptr, ...)
	end
	function META:GetSpringStiffnessLimited(...)
		return CLIB.btTranslationalLimitMotor2_getSpringStiffnessLimited(self.ptr, ...)
	end
	function META:SetUpperLimit(...)
		return CLIB.btTranslationalLimitMotor2_setUpperLimit(self.ptr, ...)
	end
	function META:GetStopCFM(...)
		return CLIB.btTranslationalLimitMotor2_getStopCFM(self.ptr, ...)
	end
	function META:GetCurrentLimitError(...)
		return CLIB.btTranslationalLimitMotor2_getCurrentLimitError(self.ptr, ...)
	end
	function META:GetBounce(...)
		return CLIB.btTranslationalLimitMotor2_getBounce(self.ptr, ...)
	end
	function META:GetStopERP(...)
		return CLIB.btTranslationalLimitMotor2_getStopERP(self.ptr, ...)
	end
	function META:SetMotorCFM(...)
		return CLIB.btTranslationalLimitMotor2_setMotorCFM(self.ptr, ...)
	end
	function META:SetCurrentLinearDiff(...)
		return CLIB.btTranslationalLimitMotor2_setCurrentLinearDiff(self.ptr, ...)
	end
	function META:GetCurrentLimitErrorHi(...)
		return CLIB.btTranslationalLimitMotor2_getCurrentLimitErrorHi(self.ptr, ...)
	end
	function META:GetCurrentLinearDiff(...)
		return CLIB.btTranslationalLimitMotor2_getCurrentLinearDiff(self.ptr, ...)
	end
	function META:GetUpperLimit(...)
		return CLIB.btTranslationalLimitMotor2_getUpperLimit(self.ptr, ...)
	end
	function META:SetLowerLimit(...)
		return CLIB.btTranslationalLimitMotor2_setLowerLimit(self.ptr, ...)
	end
	function META:SetMaxMotorForce(...)
		return CLIB.btTranslationalLimitMotor2_setMaxMotorForce(self.ptr, ...)
	end
	function META:SetMotorERP(...)
		return CLIB.btTranslationalLimitMotor2_setMotorERP(self.ptr, ...)
	end
	function META:SetSpringDamping(...)
		return CLIB.btTranslationalLimitMotor2_setSpringDamping(self.ptr, ...)
	end
	function META:SetSpringStiffness(...)
		return CLIB.btTranslationalLimitMotor2_setSpringStiffness(self.ptr, ...)
	end
	function META:SetStopCFM(...)
		return CLIB.btTranslationalLimitMotor2_setStopCFM(self.ptr, ...)
	end
	function META:SetTargetVelocity(...)
		return CLIB.btTranslationalLimitMotor2_setTargetVelocity(self.ptr, ...)
	end
	function META:GetServoTarget(...)
		return CLIB.btTranslationalLimitMotor2_getServoTarget(self.ptr, ...)
	end
	function META:SetEquilibriumPoint(...)
		return CLIB.btTranslationalLimitMotor2_setEquilibriumPoint(self.ptr, ...)
	end
	function META:IsLimited(...)
		return CLIB.btTranslationalLimitMotor2_isLimited(self.ptr, ...)
	end
end
do -- btSoftBodyWorldInfo_setAir
	local META = {}
	library.metatables.btSoftBodyWorldInfo_setAir = META
	META.__index = function(s, k) return META[k] end
	function META:Density(...)
		return CLIB.btSoftBodyWorldInfo_setAir_density(self.ptr, ...)
	end
end
do -- btPrimitiveManagerBase_is
	local META = {}
	library.metatables.btPrimitiveManagerBase_is = META
	META.__index = function(s, k) return META[k] end
	function META:Trimesh(...)
		return CLIB.btPrimitiveManagerBase_is_trimesh(self.ptr, ...)
	end
end
do -- btDbvtNodePtr_array_index
	local META = {}
	library.metatables.btDbvtNodePtr_array_index = META
	META.__index = function(s, k) return META[k] end
	function META:Of(...)
		return CLIB.btDbvtNodePtr_array_index_of(self.ptr, ...)
	end
end
do -- btSimulationIslandManager
	local META = {}
	library.metatables.btSimulationIslandManager = META
	META.__index = function(s, k) return META[k] end
	function library.CreateSimulationIslandManager(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btSimulationIslandManager_new(...)
		return self
	end
	function META:GetUnionFind(...)
		return CLIB.btSimulationIslandManager_getUnionFind(self.ptr, ...)
	end
	function META:BuildIslands(...)
		return CLIB.btSimulationIslandManager_buildIslands(self.ptr, ...)
	end
	function META:StoreIslandActivationState(...)
		return CLIB.btSimulationIslandManager_storeIslandActivationState(self.ptr, ...)
	end
	function META:GetSplitIslands(...)
		return CLIB.btSimulationIslandManager_getSplitIslands(self.ptr, ...)
	end
	function META:Delete(...)
		return CLIB.btSimulationIslandManager_delete(self.ptr, ...)
	end
	function META:SetSplitIslands(...)
		return CLIB.btSimulationIslandManager_setSplitIslands(self.ptr, ...)
	end
	function META:FindUnions(...)
		return CLIB.btSimulationIslandManager_findUnions(self.ptr, ...)
	end
	function META:BuildAndProcessIslands(...)
		return CLIB.btSimulationIslandManager_buildAndProcessIslands(self.ptr, ...)
	end
	function META:InitUnionFind(...)
		return CLIB.btSimulationIslandManager_initUnionFind(self.ptr, ...)
	end
	function META:UpdateActivationState(...)
		return CLIB.btSimulationIslandManager_updateActivationState(self.ptr, ...)
	end
end
do -- btTranslationalLimitMotor
	local META = {}
	library.metatables.btTranslationalLimitMotor = META
	META.__index = function(s, k) return META[k] end
	function library.CreateTranslationalLimitMotor(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btTranslationalLimitMotor_new(...)
		return self
	end
	function library.CreateTranslationalLimitMotor2(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btTranslationalLimitMotor_new2(...)
		return self
	end
	function META:GetAccumulatedImpulse(...)
		return CLIB.btTranslationalLimitMotor_getAccumulatedImpulse(self.ptr, ...)
	end
	function META:GetStopERP(...)
		return CLIB.btTranslationalLimitMotor_getStopERP(self.ptr, ...)
	end
	function META:SolveLinearAxis(...)
		return CLIB.btTranslationalLimitMotor_solveLinearAxis(self.ptr, ...)
	end
	function META:SetUpperLimit(...)
		return CLIB.btTranslationalLimitMotor_setUpperLimit(self.ptr, ...)
	end
	function META:GetStopCFM(...)
		return CLIB.btTranslationalLimitMotor_getStopCFM(self.ptr, ...)
	end
	function META:GetEnableMotor(...)
		return CLIB.btTranslationalLimitMotor_getEnableMotor(self.ptr, ...)
	end
	function META:SetDamping(...)
		return CLIB.btTranslationalLimitMotor_setDamping(self.ptr, ...)
	end
	function META:IsLimited(...)
		return CLIB.btTranslationalLimitMotor_isLimited(self.ptr, ...)
	end
	function META:GetLowerLimit(...)
		return CLIB.btTranslationalLimitMotor_getLowerLimit(self.ptr, ...)
	end
	function META:GetDamping(...)
		return CLIB.btTranslationalLimitMotor_getDamping(self.ptr, ...)
	end
	function META:Delete(...)
		return CLIB.btTranslationalLimitMotor_delete(self.ptr, ...)
	end
	function META:GetUpperLimit(...)
		return CLIB.btTranslationalLimitMotor_getUpperLimit(self.ptr, ...)
	end
	function META:SetNormalCFM(...)
		return CLIB.btTranslationalLimitMotor_setNormalCFM(self.ptr, ...)
	end
	function META:SetStopERP(...)
		return CLIB.btTranslationalLimitMotor_setStopERP(self.ptr, ...)
	end
	function META:GetCurrentLimitError(...)
		return CLIB.btTranslationalLimitMotor_getCurrentLimitError(self.ptr, ...)
	end
	function META:GetRestitution(...)
		return CLIB.btTranslationalLimitMotor_getRestitution(self.ptr, ...)
	end
	function META:TestLimitValue(...)
		return CLIB.btTranslationalLimitMotor_testLimitValue(self.ptr, ...)
	end
	function META:SetRestitution(...)
		return CLIB.btTranslationalLimitMotor_setRestitution(self.ptr, ...)
	end
	function META:SetAccumulatedImpulse(...)
		return CLIB.btTranslationalLimitMotor_setAccumulatedImpulse(self.ptr, ...)
	end
	function META:SetMaxMotorForce(...)
		return CLIB.btTranslationalLimitMotor_setMaxMotorForce(self.ptr, ...)
	end
	function META:GetTargetVelocity(...)
		return CLIB.btTranslationalLimitMotor_getTargetVelocity(self.ptr, ...)
	end
	function META:GetMaxMotorForce(...)
		return CLIB.btTranslationalLimitMotor_getMaxMotorForce(self.ptr, ...)
	end
	function META:GetCurrentLimit(...)
		return CLIB.btTranslationalLimitMotor_getCurrentLimit(self.ptr, ...)
	end
	function META:GetNormalCFM(...)
		return CLIB.btTranslationalLimitMotor_getNormalCFM(self.ptr, ...)
	end
	function META:SetCurrentLimitError(...)
		return CLIB.btTranslationalLimitMotor_setCurrentLimitError(self.ptr, ...)
	end
	function META:SetCurrentLinearDiff(...)
		return CLIB.btTranslationalLimitMotor_setCurrentLinearDiff(self.ptr, ...)
	end
	function META:SetLimitSoftness(...)
		return CLIB.btTranslationalLimitMotor_setLimitSoftness(self.ptr, ...)
	end
	function META:SetLowerLimit(...)
		return CLIB.btTranslationalLimitMotor_setLowerLimit(self.ptr, ...)
	end
	function META:SetStopCFM(...)
		return CLIB.btTranslationalLimitMotor_setStopCFM(self.ptr, ...)
	end
	function META:SetTargetVelocity(...)
		return CLIB.btTranslationalLimitMotor_setTargetVelocity(self.ptr, ...)
	end
	function META:NeedApplyForce(...)
		return CLIB.btTranslationalLimitMotor_needApplyForce(self.ptr, ...)
	end
	function META:GetLimitSoftness(...)
		return CLIB.btTranslationalLimitMotor_getLimitSoftness(self.ptr, ...)
	end
	function META:GetCurrentLinearDiff(...)
		return CLIB.btTranslationalLimitMotor_getCurrentLinearDiff(self.ptr, ...)
	end
end
do -- btOverlappingPairCallback
	local META = {}
	library.metatables.btOverlappingPairCallback = META
	META.__index = function(s, k) return META[k] end
	function META:Delete(...)
		return CLIB.btOverlappingPairCallback_delete(self.ptr, ...)
	end
	function META:AddOverlappingPair(...)
		return CLIB.btOverlappingPairCallback_addOverlappingPair(self.ptr, ...)
	end
	function META:RemoveOverlappingPairsContainingProxy(...)
		return CLIB.btOverlappingPairCallback_removeOverlappingPairsContainingProxy(self.ptr, ...)
	end
	function META:RemoveOverlappingPair(...)
		return CLIB.btOverlappingPairCallback_removeOverlappingPair(self.ptr, ...)
	end
end
do -- btHeightfieldTerrainShape
	local META = {}
	library.metatables.btHeightfieldTerrainShape = META
	META.__index = function(s, k) return META[k] end
	function library.CreateHeightfieldTerrainShape(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btHeightfieldTerrainShape_new2(...)
		return self
	end
	function library.CreateHeightfieldTerrainShape2(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btHeightfieldTerrainShape_new(...)
		return self
	end
	function META:SetUseDiamondSubdivision2(...)
		return CLIB.btHeightfieldTerrainShape_setUseDiamondSubdivision2(self.ptr, ...)
	end
	function META:SetUseZigzagSubdivision2(...)
		return CLIB.btHeightfieldTerrainShape_setUseZigzagSubdivision2(self.ptr, ...)
	end
	function META:SetUseDiamondSubdivision(...)
		return CLIB.btHeightfieldTerrainShape_setUseDiamondSubdivision(self.ptr, ...)
	end
	function META:SetUseZigzagSubdivision(...)
		return CLIB.btHeightfieldTerrainShape_setUseZigzagSubdivision(self.ptr, ...)
	end
end
do -- btTriangleCallbackWrapper
	local META = {}
	library.metatables.btTriangleCallbackWrapper = META
	META.__index = function(s, k) return META[k] end
	function library.CreateTriangleCallbackWrapper(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btTriangleCallbackWrapper_new(...)
		return self
	end
end
do -- btDefaultVehicleRaycaster
	local META = {}
	library.metatables.btDefaultVehicleRaycaster = META
	META.__index = function(s, k) return META[k] end
	function library.CreateDefaultVehicleRaycaster(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btDefaultVehicleRaycaster_new(...)
		return self
	end
end
do -- btConvexTriangleMeshShape
	local META = {}
	library.metatables.btConvexTriangleMeshShape = META
	META.__index = function(s, k) return META[k] end
	function library.CreateConvexTriangleMeshShape(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btConvexTriangleMeshShape_new(...)
		return self
	end
	function META:GetMeshInterface(...)
		return CLIB.btConvexTriangleMeshShape_getMeshInterface(self.ptr, ...)
	end
	function META:CalculatePrincipalAxisTransform(...)
		return CLIB.btConvexTriangleMeshShape_calculatePrincipalAxisTransform(self.ptr, ...)
	end
end
do -- btSubSimplexClosestResult
	local META = {}
	library.metatables.btSubSimplexClosestResult = META
	META.__index = function(s, k) return META[k] end
	function library.CreateSubSimplexClosestResult(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btSubSimplexClosestResult_new(...)
		return self
	end
	function META:SetBarycentricCoordinates4(...)
		return CLIB.btSubSimplexClosestResult_setBarycentricCoordinates4(self.ptr, ...)
	end
	function META:SetBarycentricCoordinates3(...)
		return CLIB.btSubSimplexClosestResult_setBarycentricCoordinates3(self.ptr, ...)
	end
	function META:IsValid(...)
		return CLIB.btSubSimplexClosestResult_isValid(self.ptr, ...)
	end
	function META:GetUsedVertices(...)
		return CLIB.btSubSimplexClosestResult_getUsedVertices(self.ptr, ...)
	end
	function META:SetUsedVertices(...)
		return CLIB.btSubSimplexClosestResult_setUsedVertices(self.ptr, ...)
	end
	function META:SetDegenerate(...)
		return CLIB.btSubSimplexClosestResult_setDegenerate(self.ptr, ...)
	end
	function META:SetClosestPointOnSimplex(...)
		return CLIB.btSubSimplexClosestResult_setClosestPointOnSimplex(self.ptr, ...)
	end
	function META:SetBarycentricCoordinates5(...)
		return CLIB.btSubSimplexClosestResult_setBarycentricCoordinates5(self.ptr, ...)
	end
	function META:SetBarycentricCoordinates2(...)
		return CLIB.btSubSimplexClosestResult_setBarycentricCoordinates2(self.ptr, ...)
	end
	function META:SetBarycentricCoordinates(...)
		return CLIB.btSubSimplexClosestResult_setBarycentricCoordinates(self.ptr, ...)
	end
	function META:GetClosestPointOnSimplex(...)
		return CLIB.btSubSimplexClosestResult_getClosestPointOnSimplex(self.ptr, ...)
	end
	function META:GetBarycentricCoords(...)
		return CLIB.btSubSimplexClosestResult_getBarycentricCoords(self.ptr, ...)
	end
	function META:Delete(...)
		return CLIB.btSubSimplexClosestResult_delete(self.ptr, ...)
	end
	function META:Reset(...)
		return CLIB.btSubSimplexClosestResult_reset(self.ptr, ...)
	end
	function META:GetDegenerate(...)
		return CLIB.btSubSimplexClosestResult_getDegenerate(self.ptr, ...)
	end
end
do -- btSoftRigidDynamicsWorld
	local META = {}
	library.metatables.btSoftRigidDynamicsWorld = META
	META.__index = function(s, k) return META[k] end
	function library.CreateSoftRigidDynamicsWorld(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btSoftRigidDynamicsWorld_new(...)
		return self
	end
	function META:RemoveSoftBody(...)
		return CLIB.btSoftRigidDynamicsWorld_removeSoftBody(self.ptr, ...)
	end
	function META:SetDrawFlags(...)
		return CLIB.btSoftRigidDynamicsWorld_setDrawFlags(self.ptr, ...)
	end
	function META:GetDrawFlags(...)
		return CLIB.btSoftRigidDynamicsWorld_getDrawFlags(self.ptr, ...)
	end
	function META:AddSoftBody2(...)
		return CLIB.btSoftRigidDynamicsWorld_addSoftBody2(self.ptr, ...)
	end
	function META:GetSoftBodyArray(...)
		return CLIB.btSoftRigidDynamicsWorld_getSoftBodyArray(self.ptr, ...)
	end
	function META:GetWorldInfo(...)
		return CLIB.btSoftRigidDynamicsWorld_getWorldInfo(self.ptr, ...)
	end
	function META:AddSoftBody3(...)
		return CLIB.btSoftRigidDynamicsWorld_addSoftBody3(self.ptr, ...)
	end
	function META:AddSoftBody(...)
		return CLIB.btSoftRigidDynamicsWorld_addSoftBody(self.ptr, ...)
	end
end
do -- btBroadphaseAabbCallback
	local META = {}
	library.metatables.btBroadphaseAabbCallback = META
	META.__index = function(s, k) return META[k] end
	function META:Delete(...)
		return CLIB.btBroadphaseAabbCallback_delete(self.ptr, ...)
	end
	function META:Process(...)
		return CLIB.btBroadphaseAabbCallback_process(self.ptr, ...)
	end
end
do -- btMultiBodyDynamicsWorld
	local META = {}
	library.metatables.btMultiBodyDynamicsWorld = META
	META.__index = function(s, k) return META[k] end
	function library.CreateMultiBodyDynamicsWorld(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btMultiBodyDynamicsWorld_new(...)
		return self
	end
	function META:ClearMultiBodyForces(...)
		return CLIB.btMultiBodyDynamicsWorld_clearMultiBodyForces(self.ptr, ...)
	end
	function META:RemoveMultiBodyConstraint(...)
		return CLIB.btMultiBodyDynamicsWorld_removeMultiBodyConstraint(self.ptr, ...)
	end
	function META:RemoveMultiBody(...)
		return CLIB.btMultiBodyDynamicsWorld_removeMultiBody(self.ptr, ...)
	end
	function META:GetNumMultiBodyConstraints(...)
		return CLIB.btMultiBodyDynamicsWorld_getNumMultiBodyConstraints(self.ptr, ...)
	end
	function META:ForwardKinematics(...)
		return CLIB.btMultiBodyDynamicsWorld_forwardKinematics(self.ptr, ...)
	end
	function META:DebugDrawMultiBodyConstraint(...)
		return CLIB.btMultiBodyDynamicsWorld_debugDrawMultiBodyConstraint(self.ptr, ...)
	end
	function META:ClearMultiBodyConstraintForces(...)
		return CLIB.btMultiBodyDynamicsWorld_clearMultiBodyConstraintForces(self.ptr, ...)
	end
	function META:GetMultiBodyConstraint(...)
		return CLIB.btMultiBodyDynamicsWorld_getMultiBodyConstraint(self.ptr, ...)
	end
	function META:IntegrateTransforms(...)
		return CLIB.btMultiBodyDynamicsWorld_integrateTransforms(self.ptr, ...)
	end
	function META:GetNumMultibodies(...)
		return CLIB.btMultiBodyDynamicsWorld_getNumMultibodies(self.ptr, ...)
	end
	function META:GetMultiBody(...)
		return CLIB.btMultiBodyDynamicsWorld_getMultiBody(self.ptr, ...)
	end
	function META:AddMultiBody(...)
		return CLIB.btMultiBodyDynamicsWorld_addMultiBody(self.ptr, ...)
	end
	function META:AddMultiBodyConstraint(...)
		return CLIB.btMultiBodyDynamicsWorld_addMultiBodyConstraint(self.ptr, ...)
	end
end
do -- btPrimitiveTriangle_clip
	local META = {}
	library.metatables.btPrimitiveTriangle_clip = META
	META.__index = function(s, k) return META[k] end
	function META:Triangle(...)
		return CLIB.btPrimitiveTriangle_clip_triangle(self.ptr, ...)
	end
end
do -- btConvexTriangleCallback
	local META = {}
	library.metatables.btConvexTriangleCallback = META
	META.__index = function(s, k) return META[k] end
	function library.CreateConvexTriangleCallback(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btConvexTriangleCallback_new(...)
		return self
	end
	function META:GetAabbMax(...)
		return CLIB.btConvexTriangleCallback_getAabbMax(self.ptr, ...)
	end
	function META:ClearWrapperData(...)
		return CLIB.btConvexTriangleCallback_clearWrapperData(self.ptr, ...)
	end
	function META:GetManifoldPtr(...)
		return CLIB.btConvexTriangleCallback_getManifoldPtr(self.ptr, ...)
	end
	function META:SetManifoldPtr(...)
		return CLIB.btConvexTriangleCallback_setManifoldPtr(self.ptr, ...)
	end
	function META:ClearCache(...)
		return CLIB.btConvexTriangleCallback_clearCache(self.ptr, ...)
	end
	function META:GetAabbMin(...)
		return CLIB.btConvexTriangleCallback_getAabbMin(self.ptr, ...)
	end
	function META:SetTriangleCount(...)
		return CLIB.btConvexTriangleCallback_setTriangleCount(self.ptr, ...)
	end
	function META:GetTriangleCount(...)
		return CLIB.btConvexTriangleCallback_getTriangleCount(self.ptr, ...)
	end
	function META:SetTimeStepAndCounters(...)
		return CLIB.btConvexTriangleCallback_setTimeStepAndCounters(self.ptr, ...)
	end
end
do -- btAABB_overlapping_trans
	local META = {}
	library.metatables.btAABB_overlapping_trans = META
	META.__index = function(s, k) return META[k] end
	function META:Cache(...)
		return CLIB.btAABB_overlapping_trans_cache(self.ptr, ...)
	end
	function META:Conservative(...)
		return CLIB.btAABB_overlapping_trans_conservative(self.ptr, ...)
	end
	function META:Conservative2(...)
		return CLIB.btAABB_overlapping_trans_conservative2(self.ptr, ...)
	end
end
do -- btQuantizedBvhTree_build
	local META = {}
	library.metatables.btQuantizedBvhTree_build = META
	META.__index = function(s, k) return META[k] end
	function META:Tree(...)
		return CLIB.btQuantizedBvhTree_build_tree(self.ptr, ...)
	end
end
do -- btActionInterfaceWrapper
	local META = {}
	library.metatables.btActionInterfaceWrapper = META
	META.__index = function(s, k) return META[k] end
	function library.CreateActionInterfaceWrapper(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btActionInterfaceWrapper_new(...)
		return self
	end
end
do -- btCollisionObjectWrapper
	local META = {}
	library.metatables.btCollisionObjectWrapper = META
	META.__index = function(s, k) return META[k] end
	function META:GetParent(...)
		return CLIB.btCollisionObjectWrapper_getParent(self.ptr, ...)
	end
	function META:SetPartId(...)
		return CLIB.btCollisionObjectWrapper_setPartId(self.ptr, ...)
	end
	function META:SetShape(...)
		return CLIB.btCollisionObjectWrapper_setShape(self.ptr, ...)
	end
	function META:GetWorldTransform(...)
		return CLIB.btCollisionObjectWrapper_getWorldTransform(self.ptr, ...)
	end
	function META:SetIndex(...)
		return CLIB.btCollisionObjectWrapper_setIndex(self.ptr, ...)
	end
	function META:SetCollisionObject(...)
		return CLIB.btCollisionObjectWrapper_setCollisionObject(self.ptr, ...)
	end
	function META:GetIndex(...)
		return CLIB.btCollisionObjectWrapper_getIndex(self.ptr, ...)
	end
	function META:SetParent(...)
		return CLIB.btCollisionObjectWrapper_setParent(self.ptr, ...)
	end
	function META:GetCollisionObject(...)
		return CLIB.btCollisionObjectWrapper_getCollisionObject(self.ptr, ...)
	end
	function META:GetPartId(...)
		return CLIB.btCollisionObjectWrapper_getPartId(self.ptr, ...)
	end
	function META:GetCollisionShape(...)
		return CLIB.btCollisionObjectWrapper_getCollisionShape(self.ptr, ...)
	end
end
do -- btPairCachingGhostObject
	local META = {}
	library.metatables.btPairCachingGhostObject = META
	META.__index = function(s, k) return META[k] end
	function library.CreatePairCachingGhostObject(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btPairCachingGhostObject_new(...)
		return self
	end
	function META:GetOverlappingPairCache(...)
		return CLIB.btPairCachingGhostObject_getOverlappingPairCache(self.ptr, ...)
	end
end
do -- btCollisionConfiguration
	local META = {}
	library.metatables.btCollisionConfiguration = META
	META.__index = function(s, k) return META[k] end
	function META:Delete(...)
		return CLIB.btCollisionConfiguration_delete(self.ptr, ...)
	end
	function META:GetCollisionAlgorithmPool(...)
		return CLIB.btCollisionConfiguration_getCollisionAlgorithmPool(self.ptr, ...)
	end
	function META:GetCollisionAlgorithmCreateFunc(...)
		return CLIB.btCollisionConfiguration_getCollisionAlgorithmCreateFunc(self.ptr, ...)
	end
	function META:GetPersistentManifoldPool(...)
		return CLIB.btCollisionConfiguration_getPersistentManifoldPool(self.ptr, ...)
	end
end
do -- btBulletXmlWorldImporter
	local META = {}
	library.metatables.btBulletXmlWorldImporter = META
	META.__index = function(s, k) return META[k] end
	function library.CreateBulletXmlWorldImporter(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btBulletXmlWorldImporter_new(...)
		return self
	end
	function META:LoadFile(...)
		return CLIB.btBulletXmlWorldImporter_loadFile(self.ptr, ...)
	end
end
do -- btOverlapFilterCallback
	local META = {}
	library.metatables.btOverlapFilterCallback = META
	META.__index = function(s, k) return META[k] end
	function META:Delete(...)
		return CLIB.btOverlapFilterCallback_delete(self.ptr, ...)
	end
	function META:NeedBroadphaseCollision(...)
		return CLIB.btOverlapFilterCallback_needBroadphaseCollision(self.ptr, ...)
	end
end
do -- btConvexConvexAlgorithm
	local META = {}
	library.metatables.btConvexConvexAlgorithm = META
	META.__index = function(s, k) return META[k] end
	function library.CreateConvexConvexAlgorithm(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btConvexConvexAlgorithm_new(...)
		return self
	end
	function META:GetManifold(...)
		return CLIB.btConvexConvexAlgorithm_getManifold(self.ptr, ...)
	end
	function META:SetLowLevelOfDetail(...)
		return CLIB.btConvexConvexAlgorithm_setLowLevelOfDetail(self.ptr, ...)
	end
end
do -- btDiscreteDynamicsWorld
	local META = {}
	library.metatables.btDiscreteDynamicsWorld = META
	function META:__index(k)
		local v

		v = META[k]
		if v ~= nil then
			return v
		end
		v = library.metatables.btDynamicsWorld.__index(self, k)
		if v ~= nil then
			return v
		end
	end
	function library.CreateDiscreteDynamicsWorld(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btDiscreteDynamicsWorld_new(...)
		return self
	end
	function META:SynchronizeSingleMotionState(...)
		return CLIB.btDiscreteDynamicsWorld_synchronizeSingleMotionState(self.ptr, ...)
	end
	function META:GetLatencyMotionStateInterpolation(...)
		return CLIB.btDiscreteDynamicsWorld_getLatencyMotionStateInterpolation(self.ptr, ...)
	end
	function META:GetCollisionWorld(...)
		return CLIB.btDiscreteDynamicsWorld_getCollisionWorld(self.ptr, ...)
	end
	function META:DebugDrawConstraint(...)
		return CLIB.btDiscreteDynamicsWorld_debugDrawConstraint(self.ptr, ...)
	end
	function META:SetApplySpeculativeContactRestitution(...)
		return CLIB.btDiscreteDynamicsWorld_setApplySpeculativeContactRestitution(self.ptr, ...)
	end
	function META:SetNumTasks(...)
		return CLIB.btDiscreteDynamicsWorld_setNumTasks(self.ptr, ...)
	end
	function META:GetSynchronizeAllMotionStates(...)
		return CLIB.btDiscreteDynamicsWorld_getSynchronizeAllMotionStates(self.ptr, ...)
	end
	function META:ApplyGravity(...)
		return CLIB.btDiscreteDynamicsWorld_applyGravity(self.ptr, ...)
	end
	function META:GetSimulationIslandManager(...)
		return CLIB.btDiscreteDynamicsWorld_getSimulationIslandManager(self.ptr, ...)
	end
	function META:GetApplySpeculativeContactRestitution(...)
		return CLIB.btDiscreteDynamicsWorld_getApplySpeculativeContactRestitution(self.ptr, ...)
	end
	function META:SetSynchronizeAllMotionStates(...)
		return CLIB.btDiscreteDynamicsWorld_setSynchronizeAllMotionStates(self.ptr, ...)
	end
	function META:SetLatencyMotionStateInterpolation(...)
		return CLIB.btDiscreteDynamicsWorld_setLatencyMotionStateInterpolation(self.ptr, ...)
	end
	function META:UpdateVehicles(...)
		return CLIB.btDiscreteDynamicsWorld_updateVehicles(self.ptr, ...)
	end
end
do -- btConvexCast_CastResult
	local META = {}
	library.metatables.btConvexCast_CastResult = META
	META.__index = function(s, k) return META[k] end
	function library.CreateConvexCast_CastResult(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btConvexCast_CastResult_new(...)
		return self
	end
	function META:DrawCoordSystem(...)
		return CLIB.btConvexCast_CastResult_drawCoordSystem(self.ptr, ...)
	end
	function META:Delete(...)
		return CLIB.btConvexCast_CastResult_delete(self.ptr, ...)
	end
	function META:GetNormal(...)
		return CLIB.btConvexCast_CastResult_getNormal(self.ptr, ...)
	end
	function META:DebugDraw(...)
		return CLIB.btConvexCast_CastResult_DebugDraw(self.ptr, ...)
	end
	function META:SetFraction(...)
		return CLIB.btConvexCast_CastResult_setFraction(self.ptr, ...)
	end
	function META:GetHitPoint(...)
		return CLIB.btConvexCast_CastResult_getHitPoint(self.ptr, ...)
	end
	function META:GetAllowedPenetration(...)
		return CLIB.btConvexCast_CastResult_getAllowedPenetration(self.ptr, ...)
	end
	function META:GetDebugDrawer(...)
		return CLIB.btConvexCast_CastResult_getDebugDrawer(self.ptr, ...)
	end
	function META:SetHitPoint(...)
		return CLIB.btConvexCast_CastResult_setHitPoint(self.ptr, ...)
	end
	function META:SetHitTransformB(...)
		return CLIB.btConvexCast_CastResult_setHitTransformB(self.ptr, ...)
	end
	function META:GetHitTransformA(...)
		return CLIB.btConvexCast_CastResult_getHitTransformA(self.ptr, ...)
	end
	function META:SetAllowedPenetration(...)
		return CLIB.btConvexCast_CastResult_setAllowedPenetration(self.ptr, ...)
	end
	function META:SetDebugDrawer(...)
		return CLIB.btConvexCast_CastResult_setDebugDrawer(self.ptr, ...)
	end
	function META:GetHitTransformB(...)
		return CLIB.btConvexCast_CastResult_getHitTransformB(self.ptr, ...)
	end
	function META:SetNormal(...)
		return CLIB.btConvexCast_CastResult_setNormal(self.ptr, ...)
	end
	function META:GetFraction(...)
		return CLIB.btConvexCast_CastResult_getFraction(self.ptr, ...)
	end
	function META:SetHitTransformA(...)
		return CLIB.btConvexCast_CastResult_setHitTransformA(self.ptr, ...)
	end
	function META:ReportFailure(...)
		return CLIB.btConvexCast_CastResult_reportFailure(self.ptr, ...)
	end
end
do -- btStridingMeshInterface
	local META = {}
	library.metatables.btStridingMeshInterface = META
	META.__index = function(s, k) return META[k] end
	function META:UnLockReadOnlyVertexBase(...)
		return CLIB.btStridingMeshInterface_unLockReadOnlyVertexBase(self.ptr, ...)
	end
	function META:CalculateAabbBruteForce(...)
		return CLIB.btStridingMeshInterface_calculateAabbBruteForce(self.ptr, ...)
	end
	function META:GetPremadeAabb(...)
		return CLIB.btStridingMeshInterface_getPremadeAabb(self.ptr, ...)
	end
	function META:PreallocateIndices(...)
		return CLIB.btStridingMeshInterface_preallocateIndices(self.ptr, ...)
	end
	function META:CalculateSerializeBufferSize(...)
		return CLIB.btStridingMeshInterface_calculateSerializeBufferSize(self.ptr, ...)
	end
	function META:GetLockedReadOnlyVertexIndexBase(...)
		return CLIB.btStridingMeshInterface_getLockedReadOnlyVertexIndexBase(self.ptr, ...)
	end
	function META:GetNumSubParts(...)
		return CLIB.btStridingMeshInterface_getNumSubParts(self.ptr, ...)
	end
	function META:UnLockVertexBase(...)
		return CLIB.btStridingMeshInterface_unLockVertexBase(self.ptr, ...)
	end
	function META:InternalProcessAllTriangles(...)
		return CLIB.btStridingMeshInterface_InternalProcessAllTriangles(self.ptr, ...)
	end
	function META:SetPremadeAabb(...)
		return CLIB.btStridingMeshInterface_setPremadeAabb(self.ptr, ...)
	end
	function META:Delete(...)
		return CLIB.btStridingMeshInterface_delete(self.ptr, ...)
	end
	function META:SetScaling(...)
		return CLIB.btStridingMeshInterface_setScaling(self.ptr, ...)
	end
	function META:Serialize(...)
		return CLIB.btStridingMeshInterface_serialize(self.ptr, ...)
	end
	function META:PreallocateVertices(...)
		return CLIB.btStridingMeshInterface_preallocateVertices(self.ptr, ...)
	end
	function META:HasPremadeAabb(...)
		return CLIB.btStridingMeshInterface_hasPremadeAabb(self.ptr, ...)
	end
	function META:GetScaling(...)
		return CLIB.btStridingMeshInterface_getScaling(self.ptr, ...)
	end
	function META:GetLockedVertexIndexBase(...)
		return CLIB.btStridingMeshInterface_getLockedVertexIndexBase(self.ptr, ...)
	end
end
do -- btConvexPointCloudShape
	local META = {}
	library.metatables.btConvexPointCloudShape = META
	META.__index = function(s, k) return META[k] end
	function library.CreateConvexPointCloudShape(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btConvexPointCloudShape_new(...)
		return self
	end
	function library.CreateConvexPointCloudShape2(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btConvexPointCloudShape_new2(...)
		return self
	end
	function META:GetNumPoints(...)
		return CLIB.btConvexPointCloudShape_getNumPoints(self.ptr, ...)
	end
	function META:GetScaledPoint(...)
		return CLIB.btConvexPointCloudShape_getScaledPoint(self.ptr, ...)
	end
	function META:GetUnscaledPoints(...)
		return CLIB.btConvexPointCloudShape_getUnscaledPoints(self.ptr, ...)
	end
	function META:SetPoints2(...)
		return CLIB.btConvexPointCloudShape_setPoints2(self.ptr, ...)
	end
	function META:SetPoints(...)
		return CLIB.btConvexPointCloudShape_setPoints(self.ptr, ...)
	end
end
do -- btGeneric6DofConstraint
	local META = {}
	library.metatables.btGeneric6DofConstraint = META
	META.__index = function(s, k) return META[k] end
	function library.CreateGeneric6DofConstraint(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btGeneric6DofConstraint_new2(...)
		return self
	end
	function library.CreateGeneric6DofConstraint2(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btGeneric6DofConstraint_new(...)
		return self
	end
	function META:SetFrames(...)
		return CLIB.btGeneric6DofConstraint_setFrames(self.ptr, ...)
	end
	function META:GetInfo1NonVirtual(...)
		return CLIB.btGeneric6DofConstraint_getInfo1NonVirtual(self.ptr, ...)
	end
	function META:GetAngle(...)
		return CLIB.btGeneric6DofConstraint_getAngle(self.ptr, ...)
	end
	function META:SetAxis(...)
		return CLIB.btGeneric6DofConstraint_setAxis(self.ptr, ...)
	end
	function META:GetInfo2NonVirtual(...)
		return CLIB.btGeneric6DofConstraint_getInfo2NonVirtual(self.ptr, ...)
	end
	function META:GetRelativePivotPosition(...)
		return CLIB.btGeneric6DofConstraint_getRelativePivotPosition(self.ptr, ...)
	end
	function META:GetAngularUpperLimit(...)
		return CLIB.btGeneric6DofConstraint_getAngularUpperLimit(self.ptr, ...)
	end
	function META:CalculateTransforms2(...)
		return CLIB.btGeneric6DofConstraint_calculateTransforms2(self.ptr, ...)
	end
	function META:UpdateRHS(...)
		return CLIB.btGeneric6DofConstraint_updateRHS(self.ptr, ...)
	end
	function META:GetCalculatedTransformB(...)
		return CLIB.btGeneric6DofConstraint_getCalculatedTransformB(self.ptr, ...)
	end
	function META:CalcAnchorPos(...)
		return CLIB.btGeneric6DofConstraint_calcAnchorPos(self.ptr, ...)
	end
	function META:GetFrameOffsetB(...)
		return CLIB.btGeneric6DofConstraint_getFrameOffsetB(self.ptr, ...)
	end
	function META:GetCalculatedTransformA(...)
		return CLIB.btGeneric6DofConstraint_getCalculatedTransformA(self.ptr, ...)
	end
	function META:SetLinearUpperLimit(...)
		return CLIB.btGeneric6DofConstraint_setLinearUpperLimit(self.ptr, ...)
	end
	function META:SetUseSolveConstraintObsolete(...)
		return CLIB.btGeneric6DofConstraint_setUseSolveConstraintObsolete(self.ptr, ...)
	end
	function META:GetTranslationalLimitMotor(...)
		return CLIB.btGeneric6DofConstraint_getTranslationalLimitMotor(self.ptr, ...)
	end
	function META:SetUseLinearReferenceFrameA(...)
		return CLIB.btGeneric6DofConstraint_setUseLinearReferenceFrameA(self.ptr, ...)
	end
	function META:SetUseFrameOffset(...)
		return CLIB.btGeneric6DofConstraint_setUseFrameOffset(self.ptr, ...)
	end
	function META:SetAngularLowerLimit(...)
		return CLIB.btGeneric6DofConstraint_setAngularLowerLimit(self.ptr, ...)
	end
	function META:GetAxis(...)
		return CLIB.btGeneric6DofConstraint_getAxis(self.ptr, ...)
	end
	function META:GetRotationalLimitMotor(...)
		return CLIB.btGeneric6DofConstraint_getRotationalLimitMotor(self.ptr, ...)
	end
	function META:GetAngularLowerLimit(...)
		return CLIB.btGeneric6DofConstraint_getAngularLowerLimit(self.ptr, ...)
	end
	function META:GetFrameOffsetA(...)
		return CLIB.btGeneric6DofConstraint_getFrameOffsetA(self.ptr, ...)
	end
	function META:GetLinearLowerLimit(...)
		return CLIB.btGeneric6DofConstraint_getLinearLowerLimit(self.ptr, ...)
	end
	function META:GetFlags(...)
		return CLIB.btGeneric6DofConstraint_getFlags(self.ptr, ...)
	end
	function META:GetUseSolveConstraintObsolete(...)
		return CLIB.btGeneric6DofConstraint_getUseSolveConstraintObsolete(self.ptr, ...)
	end
	function META:GetUseLinearReferenceFrameA(...)
		return CLIB.btGeneric6DofConstraint_getUseLinearReferenceFrameA(self.ptr, ...)
	end
	function META:IsLimited(...)
		return CLIB.btGeneric6DofConstraint_isLimited(self.ptr, ...)
	end
	function META:GetLinearUpperLimit(...)
		return CLIB.btGeneric6DofConstraint_getLinearUpperLimit(self.ptr, ...)
	end
	function META:GetUseFrameOffset(...)
		return CLIB.btGeneric6DofConstraint_getUseFrameOffset(self.ptr, ...)
	end
	function META:SetLinearLowerLimit(...)
		return CLIB.btGeneric6DofConstraint_setLinearLowerLimit(self.ptr, ...)
	end
	function META:TestAngularLimitMotor(...)
		return CLIB.btGeneric6DofConstraint_testAngularLimitMotor(self.ptr, ...)
	end
	function META:SetAngularUpperLimit(...)
		return CLIB.btGeneric6DofConstraint_setAngularUpperLimit(self.ptr, ...)
	end
	function META:SetLimit(...)
		return CLIB.btGeneric6DofConstraint_setLimit(self.ptr, ...)
	end
	function META:CalculateTransforms(...)
		return CLIB.btGeneric6DofConstraint_calculateTransforms(self.ptr, ...)
	end
end
do -- btSoftBody_AJoint_Specs
	local META = {}
	library.metatables.btSoftBody_AJoint_Specs = META
	META.__index = function(s, k) return META[k] end
	function library.CreateSoftBody_AJoint_Specs(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btSoftBody_AJoint_Specs_new(...)
		return self
	end
	function META:GetAxis(...)
		return CLIB.btSoftBody_AJoint_Specs_getAxis(self.ptr, ...)
	end
	function META:GetIcontrol(...)
		return CLIB.btSoftBody_AJoint_Specs_getIcontrol(self.ptr, ...)
	end
	function META:SetIcontrol(...)
		return CLIB.btSoftBody_AJoint_Specs_setIcontrol(self.ptr, ...)
	end
	function META:SetAxis(...)
		return CLIB.btSoftBody_AJoint_Specs_setAxis(self.ptr, ...)
	end
end
do -- btGImpactShapeInterface
	local META = {}
	library.metatables.btGImpactShapeInterface = META
	META.__index = function(s, k) return META[k] end
	function META:LockChildShapes(...)
		return CLIB.btGImpactShapeInterface_lockChildShapes(self.ptr, ...)
	end
	function META:NeedsRetrieveTetrahedrons(...)
		return CLIB.btGImpactShapeInterface_needsRetrieveTetrahedrons(self.ptr, ...)
	end
	function META:GetChildAabb(...)
		return CLIB.btGImpactShapeInterface_getChildAabb(self.ptr, ...)
	end
	function META:HasBoxSet(...)
		return CLIB.btGImpactShapeInterface_hasBoxSet(self.ptr, ...)
	end
	function META:GetBulletTriangle(...)
		return CLIB.btGImpactShapeInterface_getBulletTriangle(self.ptr, ...)
	end
	function META:GetBoxSet(...)
		return CLIB.btGImpactShapeInterface_getBoxSet(self.ptr, ...)
	end
	function META:GetLocalBox(...)
		return CLIB.btGImpactShapeInterface_getLocalBox(self.ptr, ...)
	end
	function META:UnlockChildShapes(...)
		return CLIB.btGImpactShapeInterface_unlockChildShapes(self.ptr, ...)
	end
	function META:NeedsRetrieveTriangles(...)
		return CLIB.btGImpactShapeInterface_needsRetrieveTriangles(self.ptr, ...)
	end
	function META:SetChildTransform(...)
		return CLIB.btGImpactShapeInterface_setChildTransform(self.ptr, ...)
	end
	function META:GetPrimitiveManager(...)
		return CLIB.btGImpactShapeInterface_getPrimitiveManager(self.ptr, ...)
	end
	function META:GetGImpactShapeType(...)
		return CLIB.btGImpactShapeInterface_getGImpactShapeType(self.ptr, ...)
	end
	function META:PostUpdate(...)
		return CLIB.btGImpactShapeInterface_postUpdate(self.ptr, ...)
	end
	function META:GetNumChildShapes(...)
		return CLIB.btGImpactShapeInterface_getNumChildShapes(self.ptr, ...)
	end
	function META:GetChildTransform(...)
		return CLIB.btGImpactShapeInterface_getChildTransform(self.ptr, ...)
	end
	function META:GetPrimitiveTriangle(...)
		return CLIB.btGImpactShapeInterface_getPrimitiveTriangle(self.ptr, ...)
	end
	function META:UpdateBound(...)
		return CLIB.btGImpactShapeInterface_updateBound(self.ptr, ...)
	end
	function META:GetBulletTetrahedron(...)
		return CLIB.btGImpactShapeInterface_getBulletTetrahedron(self.ptr, ...)
	end
	function META:GetChildShape(...)
		return CLIB.btGImpactShapeInterface_getChildShape(self.ptr, ...)
	end
	function META:ChildrenHasTransform(...)
		return CLIB.btGImpactShapeInterface_childrenHasTransform(self.ptr, ...)
	end
	function META:ProcessAllTrianglesRay(...)
		return CLIB.btGImpactShapeInterface_processAllTrianglesRay(self.ptr, ...)
	end
	function META:RayTest(...)
		return CLIB.btGImpactShapeInterface_rayTest(self.ptr, ...)
	end
end
do -- btDefaultSoftBodySolver
	local META = {}
	library.metatables.btDefaultSoftBodySolver = META
	META.__index = function(s, k) return META[k] end
	function library.CreateDefaultSoftBodySolver(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btDefaultSoftBodySolver_new(...)
		return self
	end
	function META:CopySoftBodyToVertexBuffer(...)
		return CLIB.btDefaultSoftBodySolver_copySoftBodyToVertexBuffer(self.ptr, ...)
	end
end
do -- btSoftBody_LJoint_Specs
	local META = {}
	library.metatables.btSoftBody_LJoint_Specs = META
	META.__index = function(s, k) return META[k] end
	function library.CreateSoftBody_LJoint_Specs(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btSoftBody_LJoint_Specs_new(...)
		return self
	end
	function META:SetPosition(...)
		return CLIB.btSoftBody_LJoint_Specs_setPosition(self.ptr, ...)
	end
	function META:GetPosition(...)
		return CLIB.btSoftBody_LJoint_Specs_getPosition(self.ptr, ...)
	end
end
do -- btPoint2PointConstraint
	local META = {}
	library.metatables.btPoint2PointConstraint = META
	META.__index = function(s, k) return META[k] end
	function library.CreatePoint2PointConstraint(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btPoint2PointConstraint_new(...)
		return self
	end
	function library.CreatePoint2PointConstraint2(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btPoint2PointConstraint_new2(...)
		return self
	end
	function META:SetPivotA(...)
		return CLIB.btPoint2PointConstraint_setPivotA(self.ptr, ...)
	end
	function META:GetSetting(...)
		return CLIB.btPoint2PointConstraint_getSetting(self.ptr, ...)
	end
	function META:GetInfo1NonVirtual(...)
		return CLIB.btPoint2PointConstraint_getInfo1NonVirtual(self.ptr, ...)
	end
	function META:GetInfo2NonVirtual(...)
		return CLIB.btPoint2PointConstraint_getInfo2NonVirtual(self.ptr, ...)
	end
	function META:GetUseSolveConstraintObsolete(...)
		return CLIB.btPoint2PointConstraint_getUseSolveConstraintObsolete(self.ptr, ...)
	end
	function META:GetFlags(...)
		return CLIB.btPoint2PointConstraint_getFlags(self.ptr, ...)
	end
	function META:GetPivotInA(...)
		return CLIB.btPoint2PointConstraint_getPivotInA(self.ptr, ...)
	end
	function META:UpdateRHS(...)
		return CLIB.btPoint2PointConstraint_updateRHS(self.ptr, ...)
	end
	function META:SetUseSolveConstraintObsolete(...)
		return CLIB.btPoint2PointConstraint_setUseSolveConstraintObsolete(self.ptr, ...)
	end
	function META:GetPivotInB(...)
		return CLIB.btPoint2PointConstraint_getPivotInB(self.ptr, ...)
	end
	function META:SetPivotB(...)
		return CLIB.btPoint2PointConstraint_setPivotB(self.ptr, ...)
	end
end
do -- btWheelInfo_RaycastInfo
	local META = {}
	library.metatables.btWheelInfo_RaycastInfo = META
	META.__index = function(s, k) return META[k] end
	function library.CreateWheelInfo_RaycastInfo(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btWheelInfo_RaycastInfo_new(...)
		return self
	end
	function META:SetWheelDirectionWS(...)
		return CLIB.btWheelInfo_RaycastInfo_setWheelDirectionWS(self.ptr, ...)
	end
	function META:SetWheelAxleWS(...)
		return CLIB.btWheelInfo_RaycastInfo_setWheelAxleWS(self.ptr, ...)
	end
	function META:SetSuspensionLength(...)
		return CLIB.btWheelInfo_RaycastInfo_setSuspensionLength(self.ptr, ...)
	end
	function META:SetIsInContact(...)
		return CLIB.btWheelInfo_RaycastInfo_setIsInContact(self.ptr, ...)
	end
	function META:SetHardPointWS(...)
		return CLIB.btWheelInfo_RaycastInfo_setHardPointWS(self.ptr, ...)
	end
	function META:SetGroundObject(...)
		return CLIB.btWheelInfo_RaycastInfo_setGroundObject(self.ptr, ...)
	end
	function META:SetContactPointWS(...)
		return CLIB.btWheelInfo_RaycastInfo_setContactPointWS(self.ptr, ...)
	end
	function META:SetContactNormalWS(...)
		return CLIB.btWheelInfo_RaycastInfo_setContactNormalWS(self.ptr, ...)
	end
	function META:GetWheelDirectionWS(...)
		return CLIB.btWheelInfo_RaycastInfo_getWheelDirectionWS(self.ptr, ...)
	end
	function META:GetSuspensionLength(...)
		return CLIB.btWheelInfo_RaycastInfo_getSuspensionLength(self.ptr, ...)
	end
	function META:GetIsInContact(...)
		return CLIB.btWheelInfo_RaycastInfo_getIsInContact(self.ptr, ...)
	end
	function META:GetHardPointWS(...)
		return CLIB.btWheelInfo_RaycastInfo_getHardPointWS(self.ptr, ...)
	end
	function META:GetGroundObject(...)
		return CLIB.btWheelInfo_RaycastInfo_getGroundObject(self.ptr, ...)
	end
	function META:GetContactNormalWS(...)
		return CLIB.btWheelInfo_RaycastInfo_getContactNormalWS(self.ptr, ...)
	end
	function META:GetContactPointWS(...)
		return CLIB.btWheelInfo_RaycastInfo_getContactPointWS(self.ptr, ...)
	end
	function META:GetWheelAxleWS(...)
		return CLIB.btWheelInfo_RaycastInfo_getWheelAxleWS(self.ptr, ...)
	end
	function META:Delete(...)
		return CLIB.btWheelInfo_RaycastInfo_delete(self.ptr, ...)
	end
end
do -- btRotationalLimitMotor2
	local META = {}
	library.metatables.btRotationalLimitMotor2 = META
	META.__index = function(s, k) return META[k] end
	function library.CreateRotationalLimitMotor2(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btRotationalLimitMotor2_new(...)
		return self
	end
	function library.CreateRotationalLimitMotor22(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btRotationalLimitMotor2_new2(...)
		return self
	end
	function META:GetSpringStiffnessLimited(...)
		return CLIB.btRotationalLimitMotor2_getSpringStiffnessLimited(self.ptr, ...)
	end
	function META:GetCurrentLimit(...)
		return CLIB.btRotationalLimitMotor2_getCurrentLimit(self.ptr, ...)
	end
	function META:GetServoTarget(...)
		return CLIB.btRotationalLimitMotor2_getServoTarget(self.ptr, ...)
	end
	function META:GetTargetVelocity(...)
		return CLIB.btRotationalLimitMotor2_getTargetVelocity(self.ptr, ...)
	end
	function META:SetSpringStiffnessLimited(...)
		return CLIB.btRotationalLimitMotor2_setSpringStiffnessLimited(self.ptr, ...)
	end
	function META:SetSpringDamping(...)
		return CLIB.btRotationalLimitMotor2_setSpringDamping(self.ptr, ...)
	end
	function META:GetBounce(...)
		return CLIB.btRotationalLimitMotor2_getBounce(self.ptr, ...)
	end
	function META:SetCurrentPosition(...)
		return CLIB.btRotationalLimitMotor2_setCurrentPosition(self.ptr, ...)
	end
	function META:GetLoLimit(...)
		return CLIB.btRotationalLimitMotor2_getLoLimit(self.ptr, ...)
	end
	function META:GetServoMotor(...)
		return CLIB.btRotationalLimitMotor2_getServoMotor(self.ptr, ...)
	end
	function META:Delete(...)
		return CLIB.btRotationalLimitMotor2_delete(self.ptr, ...)
	end
	function META:TestLimitValue(...)
		return CLIB.btRotationalLimitMotor2_testLimitValue(self.ptr, ...)
	end
	function META:GetCurrentLimitErrorHi(...)
		return CLIB.btRotationalLimitMotor2_getCurrentLimitErrorHi(self.ptr, ...)
	end
	function META:GetHiLimit(...)
		return CLIB.btRotationalLimitMotor2_getHiLimit(self.ptr, ...)
	end
	function META:SetMaxMotorForce(...)
		return CLIB.btRotationalLimitMotor2_setMaxMotorForce(self.ptr, ...)
	end
	function META:SetMotorERP(...)
		return CLIB.btRotationalLimitMotor2_setMotorERP(self.ptr, ...)
	end
	function META:SetStopCFM(...)
		return CLIB.btRotationalLimitMotor2_setStopCFM(self.ptr, ...)
	end
	function META:GetMotorCFM(...)
		return CLIB.btRotationalLimitMotor2_getMotorCFM(self.ptr, ...)
	end
	function META:GetSpringDamping(...)
		return CLIB.btRotationalLimitMotor2_getSpringDamping(self.ptr, ...)
	end
	function META:SetServoMotor(...)
		return CLIB.btRotationalLimitMotor2_setServoMotor(self.ptr, ...)
	end
	function META:SetLoLimit(...)
		return CLIB.btRotationalLimitMotor2_setLoLimit(self.ptr, ...)
	end
	function META:GetEnableSpring(...)
		return CLIB.btRotationalLimitMotor2_getEnableSpring(self.ptr, ...)
	end
	function META:GetEquilibriumPoint(...)
		return CLIB.btRotationalLimitMotor2_getEquilibriumPoint(self.ptr, ...)
	end
	function META:GetSpringStiffness(...)
		return CLIB.btRotationalLimitMotor2_getSpringStiffness(self.ptr, ...)
	end
	function META:GetMotorERP(...)
		return CLIB.btRotationalLimitMotor2_getMotorERP(self.ptr, ...)
	end
	function META:SetSpringDampingLimited(...)
		return CLIB.btRotationalLimitMotor2_setSpringDampingLimited(self.ptr, ...)
	end
	function META:GetStopERP(...)
		return CLIB.btRotationalLimitMotor2_getStopERP(self.ptr, ...)
	end
	function META:GetCurrentLimitError(...)
		return CLIB.btRotationalLimitMotor2_getCurrentLimitError(self.ptr, ...)
	end
	function META:SetCurrentLimitError(...)
		return CLIB.btRotationalLimitMotor2_setCurrentLimitError(self.ptr, ...)
	end
	function META:GetMaxMotorForce(...)
		return CLIB.btRotationalLimitMotor2_getMaxMotorForce(self.ptr, ...)
	end
	function META:GetEnableMotor(...)
		return CLIB.btRotationalLimitMotor2_getEnableMotor(self.ptr, ...)
	end
	function META:SetBounce(...)
		return CLIB.btRotationalLimitMotor2_setBounce(self.ptr, ...)
	end
	function META:SetCurrentLimit(...)
		return CLIB.btRotationalLimitMotor2_setCurrentLimit(self.ptr, ...)
	end
	function META:SetCurrentLimitErrorHi(...)
		return CLIB.btRotationalLimitMotor2_setCurrentLimitErrorHi(self.ptr, ...)
	end
	function META:SetEnableMotor(...)
		return CLIB.btRotationalLimitMotor2_setEnableMotor(self.ptr, ...)
	end
	function META:SetHiLimit(...)
		return CLIB.btRotationalLimitMotor2_setHiLimit(self.ptr, ...)
	end
	function META:SetMotorCFM(...)
		return CLIB.btRotationalLimitMotor2_setMotorCFM(self.ptr, ...)
	end
	function META:SetServoTarget(...)
		return CLIB.btRotationalLimitMotor2_setServoTarget(self.ptr, ...)
	end
	function META:SetTargetVelocity(...)
		return CLIB.btRotationalLimitMotor2_setTargetVelocity(self.ptr, ...)
	end
	function META:SetSpringStiffness(...)
		return CLIB.btRotationalLimitMotor2_setSpringStiffness(self.ptr, ...)
	end
	function META:GetSpringDampingLimited(...)
		return CLIB.btRotationalLimitMotor2_getSpringDampingLimited(self.ptr, ...)
	end
	function META:SetEquilibriumPoint(...)
		return CLIB.btRotationalLimitMotor2_setEquilibriumPoint(self.ptr, ...)
	end
	function META:SetStopERP(...)
		return CLIB.btRotationalLimitMotor2_setStopERP(self.ptr, ...)
	end
	function META:GetStopCFM(...)
		return CLIB.btRotationalLimitMotor2_getStopCFM(self.ptr, ...)
	end
	function META:GetCurrentPosition(...)
		return CLIB.btRotationalLimitMotor2_getCurrentPosition(self.ptr, ...)
	end
	function META:IsLimited(...)
		return CLIB.btRotationalLimitMotor2_isLimited(self.ptr, ...)
	end
	function META:SetEnableSpring(...)
		return CLIB.btRotationalLimitMotor2_setEnableSpring(self.ptr, ...)
	end
end
do -- btMultiBodyLinkCollider
	local META = {}
	library.metatables.btMultiBodyLinkCollider = META
	META.__index = function(s, k) return META[k] end
	function library.CreateMultiBodyLinkCollider(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btMultiBodyLinkCollider_new(...)
		return self
	end
	function META:SetMultiBody(...)
		return CLIB.btMultiBodyLinkCollider_setMultiBody(self.ptr, ...)
	end
	function META:SetLink(...)
		return CLIB.btMultiBodyLinkCollider_setLink(self.ptr, ...)
	end
	function META:GetMultiBody(...)
		return CLIB.btMultiBodyLinkCollider_getMultiBody(self.ptr, ...)
	end
	function META:GetLink(...)
		return CLIB.btMultiBodyLinkCollider_getLink(self.ptr, ...)
	end
	function META:Upcast(...)
		return CLIB.btMultiBodyLinkCollider_upcast(self.ptr, ...)
	end
end
do -- btAABB_collide_triangle
	local META = {}
	library.metatables.btAABB_collide_triangle = META
	META.__index = function(s, k) return META[k] end
	function META:Exact(...)
		return CLIB.btAABB_collide_triangle_exact(self.ptr, ...)
	end
end
do -- btBroadphaseRayCallback
	local META = {}
	library.metatables.btBroadphaseRayCallback = META
	META.__index = function(s, k) return META[k] end
	function META:GetSigns(...)
		return CLIB.btBroadphaseRayCallback_getSigns(self.ptr, ...)
	end
	function META:GetRayDirectionInverse(...)
		return CLIB.btBroadphaseRayCallback_getRayDirectionInverse(self.ptr, ...)
	end
	function META:SetRayDirectionInverse(...)
		return CLIB.btBroadphaseRayCallback_setRayDirectionInverse(self.ptr, ...)
	end
end
do -- btContactSolverInfoData
	local META = {}
	library.metatables.btContactSolverInfoData = META
	META.__index = function(s, k) return META[k] end
	function library.CreateContactSolverInfoData(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btContactSolverInfoData_new(...)
		return self
	end
	function META:GetSingleAxisRollingFrictionThreshold(...)
		return CLIB.btContactSolverInfoData_getSingleAxisRollingFrictionThreshold(self.ptr, ...)
	end
	function META:SetFriction(...)
		return CLIB.btContactSolverInfoData_setFriction(self.ptr, ...)
	end
	function META:SetSolverMode(...)
		return CLIB.btContactSolverInfoData_setSolverMode(self.ptr, ...)
	end
	function META:GetTimeStep(...)
		return CLIB.btContactSolverInfoData_getTimeStep(self.ptr, ...)
	end
	function META:SetDamping(...)
		return CLIB.btContactSolverInfoData_setDamping(self.ptr, ...)
	end
	function META:GetWarmstartingFactor(...)
		return CLIB.btContactSolverInfoData_getWarmstartingFactor(self.ptr, ...)
	end
	function META:SetMinimumSolverBatchSize(...)
		return CLIB.btContactSolverInfoData_setMinimumSolverBatchSize(self.ptr, ...)
	end
	function META:GetTau(...)
		return CLIB.btContactSolverInfoData_getTau(self.ptr, ...)
	end
	function META:SetGlobalCfm(...)
		return CLIB.btContactSolverInfoData_setGlobalCfm(self.ptr, ...)
	end
	function META:Delete(...)
		return CLIB.btContactSolverInfoData_delete(self.ptr, ...)
	end
	function META:GetMinimumSolverBatchSize(...)
		return CLIB.btContactSolverInfoData_getMinimumSolverBatchSize(self.ptr, ...)
	end
	function META:GetSor(...)
		return CLIB.btContactSolverInfoData_getSor(self.ptr, ...)
	end
	function META:GetFrictionCfm(...)
		return CLIB.btContactSolverInfoData_getFrictionCfm(self.ptr, ...)
	end
	function META:SetMaxGyroscopicForce(...)
		return CLIB.btContactSolverInfoData_setMaxGyroscopicForce(self.ptr, ...)
	end
	function META:SetFrictionErp(...)
		return CLIB.btContactSolverInfoData_setFrictionErp(self.ptr, ...)
	end
	function META:GetFriction(...)
		return CLIB.btContactSolverInfoData_getFriction(self.ptr, ...)
	end
	function META:GetLinearSlop(...)
		return CLIB.btContactSolverInfoData_getLinearSlop(self.ptr, ...)
	end
	function META:GetErp2(...)
		return CLIB.btContactSolverInfoData_getErp2(self.ptr, ...)
	end
	function META:GetRestingContactRestitutionThreshold(...)
		return CLIB.btContactSolverInfoData_getRestingContactRestitutionThreshold(self.ptr, ...)
	end
	function META:GetSplitImpulseTurnErp(...)
		return CLIB.btContactSolverInfoData_getSplitImpulseTurnErp(self.ptr, ...)
	end
	function META:GetNumIterations(...)
		return CLIB.btContactSolverInfoData_getNumIterations(self.ptr, ...)
	end
	function META:SetSplitImpulsePenetrationThreshold(...)
		return CLIB.btContactSolverInfoData_setSplitImpulsePenetrationThreshold(self.ptr, ...)
	end
	function META:SetRestitution(...)
		return CLIB.btContactSolverInfoData_setRestitution(self.ptr, ...)
	end
	function META:GetMaxErrorReduction(...)
		return CLIB.btContactSolverInfoData_getMaxErrorReduction(self.ptr, ...)
	end
	function META:SetSor(...)
		return CLIB.btContactSolverInfoData_setSor(self.ptr, ...)
	end
	function META:SetSplitImpulseTurnErp(...)
		return CLIB.btContactSolverInfoData_setSplitImpulseTurnErp(self.ptr, ...)
	end
	function META:SetMaxErrorReduction(...)
		return CLIB.btContactSolverInfoData_setMaxErrorReduction(self.ptr, ...)
	end
	function META:SetErp(...)
		return CLIB.btContactSolverInfoData_setErp(self.ptr, ...)
	end
	function META:GetRestitution(...)
		return CLIB.btContactSolverInfoData_getRestitution(self.ptr, ...)
	end
	function META:GetSplitImpulse(...)
		return CLIB.btContactSolverInfoData_getSplitImpulse(self.ptr, ...)
	end
	function META:SetWarmstartingFactor(...)
		return CLIB.btContactSolverInfoData_setWarmstartingFactor(self.ptr, ...)
	end
	function META:SetRestingContactRestitutionThreshold(...)
		return CLIB.btContactSolverInfoData_setRestingContactRestitutionThreshold(self.ptr, ...)
	end
	function META:SetLinearSlop(...)
		return CLIB.btContactSolverInfoData_setLinearSlop(self.ptr, ...)
	end
	function META:GetSplitImpulsePenetrationThreshold(...)
		return CLIB.btContactSolverInfoData_getSplitImpulsePenetrationThreshold(self.ptr, ...)
	end
	function META:GetSolverMode(...)
		return CLIB.btContactSolverInfoData_getSolverMode(self.ptr, ...)
	end
	function META:GetMaxGyroscopicForce(...)
		return CLIB.btContactSolverInfoData_getMaxGyroscopicForce(self.ptr, ...)
	end
	function META:GetDamping(...)
		return CLIB.btContactSolverInfoData_getDamping(self.ptr, ...)
	end
	function META:SetErp2(...)
		return CLIB.btContactSolverInfoData_setErp2(self.ptr, ...)
	end
	function META:SetTau(...)
		return CLIB.btContactSolverInfoData_setTau(self.ptr, ...)
	end
	function META:GetFrictionErp(...)
		return CLIB.btContactSolverInfoData_getFrictionErp(self.ptr, ...)
	end
	function META:SetFrictionCfm(...)
		return CLIB.btContactSolverInfoData_setFrictionCfm(self.ptr, ...)
	end
	function META:SetTimeStep(...)
		return CLIB.btContactSolverInfoData_setTimeStep(self.ptr, ...)
	end
	function META:SetSplitImpulse(...)
		return CLIB.btContactSolverInfoData_setSplitImpulse(self.ptr, ...)
	end
	function META:SetNumIterations(...)
		return CLIB.btContactSolverInfoData_setNumIterations(self.ptr, ...)
	end
	function META:GetErp(...)
		return CLIB.btContactSolverInfoData_getErp(self.ptr, ...)
	end
	function META:GetGlobalCfm(...)
		return CLIB.btContactSolverInfoData_getGlobalCfm(self.ptr, ...)
	end
	function META:SetSingleAxisRollingFrictionThreshold(...)
		return CLIB.btContactSolverInfoData_setSingleAxisRollingFrictionThreshold(self.ptr, ...)
	end
end
do -- btPolyhedralConvexShape
	local META = {}
	library.metatables.btPolyhedralConvexShape = META
	META.__index = function(s, k) return META[k] end
	function META:GetConvexPolyhedron(...)
		return CLIB.btPolyhedralConvexShape_getConvexPolyhedron(self.ptr, ...)
	end
	function META:GetNumPlanes(...)
		return CLIB.btPolyhedralConvexShape_getNumPlanes(self.ptr, ...)
	end
	function META:GetPlane(...)
		return CLIB.btPolyhedralConvexShape_getPlane(self.ptr, ...)
	end
	function META:GetEdge(...)
		return CLIB.btPolyhedralConvexShape_getEdge(self.ptr, ...)
	end
	function META:GetNumVertices(...)
		return CLIB.btPolyhedralConvexShape_getNumVertices(self.ptr, ...)
	end
	function META:IsInside(...)
		return CLIB.btPolyhedralConvexShape_isInside(self.ptr, ...)
	end
	function META:GetNumEdges(...)
		return CLIB.btPolyhedralConvexShape_getNumEdges(self.ptr, ...)
	end
	function META:InitializePolyhedralFeatures(...)
		return CLIB.btPolyhedralConvexShape_initializePolyhedralFeatures(self.ptr, ...)
	end
	function META:GetVertex(...)
		return CLIB.btPolyhedralConvexShape_getVertex(self.ptr, ...)
	end
end
do -- btNNCGConstraintSolver
	local META = {}
	library.metatables.btNNCGConstraintSolver = META
	META.__index = function(s, k) return META[k] end
	function library.CreateNNCGConstraintSolver(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btNNCGConstraintSolver_new(...)
		return self
	end
	function META:SetOnlyForNoneContact(...)
		return CLIB.btNNCGConstraintSolver_setOnlyForNoneContact(self.ptr, ...)
	end
	function META:GetOnlyForNoneContact(...)
		return CLIB.btNNCGConstraintSolver_getOnlyForNoneContact(self.ptr, ...)
	end
end
do -- btGImpactMeshShapePart
	local META = {}
	library.metatables.btGImpactMeshShapePart = META
	META.__index = function(s, k) return META[k] end
	function library.CreateGImpactMeshShapePart(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btGImpactMeshShapePart_new2(...)
		return self
	end
	function library.CreateGImpactMeshShapePart2(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btGImpactMeshShapePart_new(...)
		return self
	end
	function META:GetVertex(...)
		return CLIB.btGImpactMeshShapePart_getVertex(self.ptr, ...)
	end
	function META:GetTrimeshPrimitiveManager(...)
		return CLIB.btGImpactMeshShapePart_getTrimeshPrimitiveManager(self.ptr, ...)
	end
	function META:GetPart(...)
		return CLIB.btGImpactMeshShapePart_getPart(self.ptr, ...)
	end
	function META:GetVertexCount(...)
		return CLIB.btGImpactMeshShapePart_getVertexCount(self.ptr, ...)
	end
end
do -- btRotationalLimitMotor
	local META = {}
	library.metatables.btRotationalLimitMotor = META
	META.__index = function(s, k) return META[k] end
	function library.CreateRotationalLimitMotor(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btRotationalLimitMotor_new(...)
		return self
	end
	function library.CreateRotationalLimitMotor2(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btRotationalLimitMotor_new2(...)
		return self
	end
	function META:SetLimitSoftness(...)
		return CLIB.btRotationalLimitMotor_setLimitSoftness(self.ptr, ...)
	end
	function META:SetAccumulatedImpulse(...)
		return CLIB.btRotationalLimitMotor_setAccumulatedImpulse(self.ptr, ...)
	end
	function META:GetMaxLimitForce(...)
		return CLIB.btRotationalLimitMotor_getMaxLimitForce(self.ptr, ...)
	end
	function META:TestLimitValue(...)
		return CLIB.btRotationalLimitMotor_testLimitValue(self.ptr, ...)
	end
	function META:GetMaxMotorForce(...)
		return CLIB.btRotationalLimitMotor_getMaxMotorForce(self.ptr, ...)
	end
	function META:IsLimited(...)
		return CLIB.btRotationalLimitMotor_isLimited(self.ptr, ...)
	end
	function META:SetCurrentPosition(...)
		return CLIB.btRotationalLimitMotor_setCurrentPosition(self.ptr, ...)
	end
	function META:SetStopERP(...)
		return CLIB.btRotationalLimitMotor_setStopERP(self.ptr, ...)
	end
	function META:GetStopCFM(...)
		return CLIB.btRotationalLimitMotor_getStopCFM(self.ptr, ...)
	end
	function META:GetHiLimit(...)
		return CLIB.btRotationalLimitMotor_getHiLimit(self.ptr, ...)
	end
	function META:Delete(...)
		return CLIB.btRotationalLimitMotor_delete(self.ptr, ...)
	end
	function META:GetAccumulatedImpulse(...)
		return CLIB.btRotationalLimitMotor_getAccumulatedImpulse(self.ptr, ...)
	end
	function META:GetNormalCFM(...)
		return CLIB.btRotationalLimitMotor_getNormalCFM(self.ptr, ...)
	end
	function META:SetDamping(...)
		return CLIB.btRotationalLimitMotor_setDamping(self.ptr, ...)
	end
	function META:GetStopERP(...)
		return CLIB.btRotationalLimitMotor_getStopERP(self.ptr, ...)
	end
	function META:GetBounce(...)
		return CLIB.btRotationalLimitMotor_getBounce(self.ptr, ...)
	end
	function META:GetCurrentPosition(...)
		return CLIB.btRotationalLimitMotor_getCurrentPosition(self.ptr, ...)
	end
	function META:GetCurrentLimitError(...)
		return CLIB.btRotationalLimitMotor_getCurrentLimitError(self.ptr, ...)
	end
	function META:GetDamping(...)
		return CLIB.btRotationalLimitMotor_getDamping(self.ptr, ...)
	end
	function META:SetNormalCFM(...)
		return CLIB.btRotationalLimitMotor_setNormalCFM(self.ptr, ...)
	end
	function META:SetMaxLimitForce(...)
		return CLIB.btRotationalLimitMotor_setMaxLimitForce(self.ptr, ...)
	end
	function META:SetCurrentLimit(...)
		return CLIB.btRotationalLimitMotor_setCurrentLimit(self.ptr, ...)
	end
	function META:GetTargetVelocity(...)
		return CLIB.btRotationalLimitMotor_getTargetVelocity(self.ptr, ...)
	end
	function META:GetCurrentLimit(...)
		return CLIB.btRotationalLimitMotor_getCurrentLimit(self.ptr, ...)
	end
	function META:SetTargetVelocity(...)
		return CLIB.btRotationalLimitMotor_setTargetVelocity(self.ptr, ...)
	end
	function META:GetLimitSoftness(...)
		return CLIB.btRotationalLimitMotor_getLimitSoftness(self.ptr, ...)
	end
	function META:GetLoLimit(...)
		return CLIB.btRotationalLimitMotor_getLoLimit(self.ptr, ...)
	end
	function META:NeedApplyTorques(...)
		return CLIB.btRotationalLimitMotor_needApplyTorques(self.ptr, ...)
	end
	function META:SolveAngularLimits(...)
		return CLIB.btRotationalLimitMotor_solveAngularLimits(self.ptr, ...)
	end
	function META:SetHiLimit(...)
		return CLIB.btRotationalLimitMotor_setHiLimit(self.ptr, ...)
	end
	function META:SetEnableMotor(...)
		return CLIB.btRotationalLimitMotor_setEnableMotor(self.ptr, ...)
	end
	function META:SetLoLimit(...)
		return CLIB.btRotationalLimitMotor_setLoLimit(self.ptr, ...)
	end
	function META:SetMaxMotorForce(...)
		return CLIB.btRotationalLimitMotor_setMaxMotorForce(self.ptr, ...)
	end
	function META:SetStopCFM(...)
		return CLIB.btRotationalLimitMotor_setStopCFM(self.ptr, ...)
	end
	function META:SetCurrentLimitError(...)
		return CLIB.btRotationalLimitMotor_setCurrentLimitError(self.ptr, ...)
	end
	function META:SetBounce(...)
		return CLIB.btRotationalLimitMotor_setBounce(self.ptr, ...)
	end
	function META:GetEnableMotor(...)
		return CLIB.btRotationalLimitMotor_getEnableMotor(self.ptr, ...)
	end
end
do -- btVoronoiSimplexSolver
	local META = {}
	library.metatables.btVoronoiSimplexSolver = META
	META.__index = function(s, k) return META[k] end
	function library.CreateVoronoiSimplexSolver(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btVoronoiSimplexSolver_new(...)
		return self
	end
	function META:GetSimplex(...)
		return CLIB.btVoronoiSimplexSolver_getSimplex(self.ptr, ...)
	end
	function META:UpdateClosestVectorAndPoints(...)
		return CLIB.btVoronoiSimplexSolver_updateClosestVectorAndPoints(self.ptr, ...)
	end
	function META:SetNumVertices(...)
		return CLIB.btVoronoiSimplexSolver_setNumVertices(self.ptr, ...)
	end
	function META:SetNeedsUpdate(...)
		return CLIB.btVoronoiSimplexSolver_setNeedsUpdate(self.ptr, ...)
	end
	function META:SetLastW(...)
		return CLIB.btVoronoiSimplexSolver_setLastW(self.ptr, ...)
	end
	function META:SetCachedV(...)
		return CLIB.btVoronoiSimplexSolver_setCachedV(self.ptr, ...)
	end
	function META:SetCachedP2(...)
		return CLIB.btVoronoiSimplexSolver_setCachedP2(self.ptr, ...)
	end
	function META:SetCachedP1(...)
		return CLIB.btVoronoiSimplexSolver_setCachedP1(self.ptr, ...)
	end
	function META:SetCachedBC(...)
		return CLIB.btVoronoiSimplexSolver_setCachedBC(self.ptr, ...)
	end
	function META:Reset(...)
		return CLIB.btVoronoiSimplexSolver_reset(self.ptr, ...)
	end
	function META:RemoveVertex(...)
		return CLIB.btVoronoiSimplexSolver_removeVertex(self.ptr, ...)
	end
	function META:ReduceVertices(...)
		return CLIB.btVoronoiSimplexSolver_reduceVertices(self.ptr, ...)
	end
	function META:PointOutsideOfPlane(...)
		return CLIB.btVoronoiSimplexSolver_pointOutsideOfPlane(self.ptr, ...)
	end
	function META:NumVertices(...)
		return CLIB.btVoronoiSimplexSolver_numVertices(self.ptr, ...)
	end
	function META:MaxVertex(...)
		return CLIB.btVoronoiSimplexSolver_maxVertex(self.ptr, ...)
	end
	function META:InSimplex(...)
		return CLIB.btVoronoiSimplexSolver_inSimplex(self.ptr, ...)
	end
	function META:GetSimplexPointsQ(...)
		return CLIB.btVoronoiSimplexSolver_getSimplexPointsQ(self.ptr, ...)
	end
	function META:GetSimplexPointsP(...)
		return CLIB.btVoronoiSimplexSolver_getSimplexPointsP(self.ptr, ...)
	end
	function META:GetNeedsUpdate(...)
		return CLIB.btVoronoiSimplexSolver_getNeedsUpdate(self.ptr, ...)
	end
	function META:GetCachedValidClosest(...)
		return CLIB.btVoronoiSimplexSolver_getCachedValidClosest(self.ptr, ...)
	end
	function META:GetCachedV(...)
		return CLIB.btVoronoiSimplexSolver_getCachedV(self.ptr, ...)
	end
	function META:FullSimplex(...)
		return CLIB.btVoronoiSimplexSolver_fullSimplex(self.ptr, ...)
	end
	function META:EmptySimplex(...)
		return CLIB.btVoronoiSimplexSolver_emptySimplex(self.ptr, ...)
	end
	function META:ClosestPtPointTetrahedron(...)
		return CLIB.btVoronoiSimplexSolver_closestPtPointTetrahedron(self.ptr, ...)
	end
	function META:AddVertex(...)
		return CLIB.btVoronoiSimplexSolver_addVertex(self.ptr, ...)
	end
	function META:GetCachedBC(...)
		return CLIB.btVoronoiSimplexSolver_getCachedBC(self.ptr, ...)
	end
	function META:ClosestPtPointTriangle(...)
		return CLIB.btVoronoiSimplexSolver_closestPtPointTriangle(self.ptr, ...)
	end
	function META:GetLastW(...)
		return CLIB.btVoronoiSimplexSolver_getLastW(self.ptr, ...)
	end
	function META:SetEqualVertexThreshold(...)
		return CLIB.btVoronoiSimplexSolver_setEqualVertexThreshold(self.ptr, ...)
	end
	function META:SetCachedValidClosest(...)
		return CLIB.btVoronoiSimplexSolver_setCachedValidClosest(self.ptr, ...)
	end
	function META:GetEqualVertexThreshold(...)
		return CLIB.btVoronoiSimplexSolver_getEqualVertexThreshold(self.ptr, ...)
	end
	function META:GetSimplexVectorW(...)
		return CLIB.btVoronoiSimplexSolver_getSimplexVectorW(self.ptr, ...)
	end
	function META:GetCachedP1(...)
		return CLIB.btVoronoiSimplexSolver_getCachedP1(self.ptr, ...)
	end
	function META:GetCachedP2(...)
		return CLIB.btVoronoiSimplexSolver_getCachedP2(self.ptr, ...)
	end
	function META:Closest(...)
		return CLIB.btVoronoiSimplexSolver_closest(self.ptr, ...)
	end
	function META:GetNumVertices(...)
		return CLIB.btVoronoiSimplexSolver_getNumVertices(self.ptr, ...)
	end
	function META:Delete(...)
		return CLIB.btVoronoiSimplexSolver_delete(self.ptr, ...)
	end
end
do -- btMultiBodyPoint2Point
	local META = {}
	library.metatables.btMultiBodyPoint2Point = META
	META.__index = function(s, k) return META[k] end
	function library.CreateMultiBodyPoint2Point(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btMultiBodyPoint2Point_new(...)
		return self
	end
	function library.CreateMultiBodyPoint2Point2(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btMultiBodyPoint2Point_new2(...)
		return self
	end
	function META:SetPivotInB(...)
		return CLIB.btMultiBodyPoint2Point_setPivotInB(self.ptr, ...)
	end
	function META:GetPivotInB(...)
		return CLIB.btMultiBodyPoint2Point_getPivotInB(self.ptr, ...)
	end
end
do -- btGImpactCompoundShape
	local META = {}
	library.metatables.btGImpactCompoundShape = META
	META.__index = function(s, k) return META[k] end
	function library.CreateGImpactCompoundShape(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btGImpactCompoundShape_new(...)
		return self
	end
	function META:AddChildShape2(...)
		return CLIB.btGImpactCompoundShape_addChildShape2(self.ptr, ...)
	end
	function META:GetCompoundPrimitiveManager(...)
		return CLIB.btGImpactCompoundShape_getCompoundPrimitiveManager(self.ptr, ...)
	end
	function META:AddChildShape(...)
		return CLIB.btGImpactCompoundShape_addChildShape(self.ptr, ...)
	end
end
do -- btPrimitiveManagerBase
	local META = {}
	library.metatables.btPrimitiveManagerBase = META
	META.__index = function(s, k) return META[k] end
	function META:Delete(...)
		return CLIB.btPrimitiveManagerBase_delete(self.ptr, ...)
	end
end
do -- btSoftBody_SolverState
	local META = {}
	library.metatables.btSoftBody_SolverState = META
	META.__index = function(s, k) return META[k] end
	function META:SetUpdmrg(...)
		return CLIB.btSoftBody_SolverState_setUpdmrg(self.ptr, ...)
	end
	function META:SetSdt(...)
		return CLIB.btSoftBody_SolverState_setSdt(self.ptr, ...)
	end
	function META:SetVelmrg(...)
		return CLIB.btSoftBody_SolverState_setVelmrg(self.ptr, ...)
	end
	function META:SetRadmrg(...)
		return CLIB.btSoftBody_SolverState_setRadmrg(self.ptr, ...)
	end
	function META:SetIsdt(...)
		return CLIB.btSoftBody_SolverState_setIsdt(self.ptr, ...)
	end
	function META:GetVelmrg(...)
		return CLIB.btSoftBody_SolverState_getVelmrg(self.ptr, ...)
	end
	function META:GetUpdmrg(...)
		return CLIB.btSoftBody_SolverState_getUpdmrg(self.ptr, ...)
	end
	function META:GetSdt(...)
		return CLIB.btSoftBody_SolverState_getSdt(self.ptr, ...)
	end
	function META:GetRadmrg(...)
		return CLIB.btSoftBody_SolverState_getRadmrg(self.ptr, ...)
	end
	function META:GetIsdt(...)
		return CLIB.btSoftBody_SolverState_getIsdt(self.ptr, ...)
	end
end
do -- btBvhTriangleMeshShape
	local META = {}
	library.metatables.btBvhTriangleMeshShape = META
	META.__index = function(s, k) return META[k] end
	function library.CreateBvhTriangleMeshShape(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btBvhTriangleMeshShape_new2(...)
		return self
	end
	function library.CreateBvhTriangleMeshShape2(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btBvhTriangleMeshShape_new(...)
		return self
	end
	function META:SetOptimizedBvh(...)
		return CLIB.btBvhTriangleMeshShape_setOptimizedBvh(self.ptr, ...)
	end
	function META:SerializeSingleTriangleInfoMap(...)
		return CLIB.btBvhTriangleMeshShape_serializeSingleTriangleInfoMap(self.ptr, ...)
	end
	function META:PerformRaycast(...)
		return CLIB.btBvhTriangleMeshShape_performRaycast(self.ptr, ...)
	end
	function META:GetOwnsBvh(...)
		return CLIB.btBvhTriangleMeshShape_getOwnsBvh(self.ptr, ...)
	end
	function META:SerializeSingleBvh(...)
		return CLIB.btBvhTriangleMeshShape_serializeSingleBvh(self.ptr, ...)
	end
	function META:RefitTree(...)
		return CLIB.btBvhTriangleMeshShape_refitTree(self.ptr, ...)
	end
	function META:GetTriangleInfoMap(...)
		return CLIB.btBvhTriangleMeshShape_getTriangleInfoMap(self.ptr, ...)
	end
	function META:BuildOptimizedBvh(...)
		return CLIB.btBvhTriangleMeshShape_buildOptimizedBvh(self.ptr, ...)
	end
	function META:PerformConvexcast(...)
		return CLIB.btBvhTriangleMeshShape_performConvexcast(self.ptr, ...)
	end
	function META:UsesQuantizedAabbCompression(...)
		return CLIB.btBvhTriangleMeshShape_usesQuantizedAabbCompression(self.ptr, ...)
	end
	function META:GetOptimizedBvh(...)
		return CLIB.btBvhTriangleMeshShape_getOptimizedBvh(self.ptr, ...)
	end
	function META:SetTriangleInfoMap(...)
		return CLIB.btBvhTriangleMeshShape_setTriangleInfoMap(self.ptr, ...)
	end
	function META:SetOptimizedBvh2(...)
		return CLIB.btBvhTriangleMeshShape_setOptimizedBvh2(self.ptr, ...)
	end
	function META:PartialRefitTree(...)
		return CLIB.btBvhTriangleMeshShape_partialRefitTree(self.ptr, ...)
	end
end
do -- btOverlappingPairCache
	local META = {}
	library.metatables.btOverlappingPairCache = META
	META.__index = function(s, k) return META[k] end
	function META:SetInternalGhostPairCallback(...)
		return CLIB.btOverlappingPairCache_setInternalGhostPairCallback(self.ptr, ...)
	end
	function META:HasDeferredRemoval(...)
		return CLIB.btOverlappingPairCache_hasDeferredRemoval(self.ptr, ...)
	end
	function META:GetOverlappingPairArray(...)
		return CLIB.btOverlappingPairCache_getOverlappingPairArray(self.ptr, ...)
	end
	function META:GetNumOverlappingPairs(...)
		return CLIB.btOverlappingPairCache_getNumOverlappingPairs(self.ptr, ...)
	end
	function META:CleanProxyFromPairs(...)
		return CLIB.btOverlappingPairCache_cleanProxyFromPairs(self.ptr, ...)
	end
	function META:SortOverlappingPairs(...)
		return CLIB.btOverlappingPairCache_sortOverlappingPairs(self.ptr, ...)
	end
	function META:FindPair(...)
		return CLIB.btOverlappingPairCache_findPair(self.ptr, ...)
	end
	function META:SetOverlapFilterCallback(...)
		return CLIB.btOverlappingPairCache_setOverlapFilterCallback(self.ptr, ...)
	end
	function META:ProcessAllOverlappingPairs(...)
		return CLIB.btOverlappingPairCache_processAllOverlappingPairs(self.ptr, ...)
	end
	function META:CleanOverlappingPair(...)
		return CLIB.btOverlappingPairCache_cleanOverlappingPair(self.ptr, ...)
	end
	function META:GetOverlappingPairArrayPtr(...)
		return CLIB.btOverlappingPairCache_getOverlappingPairArrayPtr(self.ptr, ...)
	end
end
do -- btSoftBodySolverOutput
	local META = {}
	library.metatables.btSoftBodySolverOutput = META
	META.__index = function(s, k) return META[k] end
	function META:Delete(...)
		return CLIB.btSoftBodySolverOutput_delete(self.ptr, ...)
	end
end
do -- btSoftBody_Joint_Specs
	local META = {}
	library.metatables.btSoftBody_Joint_Specs = META
	META.__index = function(s, k) return META[k] end
	function library.CreateSoftBody_Joint_Specs(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btSoftBody_Joint_Specs_new(...)
		return self
	end
	function META:SetCfm(...)
		return CLIB.btSoftBody_Joint_Specs_setCfm(self.ptr, ...)
	end
	function META:GetErp(...)
		return CLIB.btSoftBody_Joint_Specs_getErp(self.ptr, ...)
	end
	function META:GetCfm(...)
		return CLIB.btSoftBody_Joint_Specs_getCfm(self.ptr, ...)
	end
	function META:Delete(...)
		return CLIB.btSoftBody_Joint_Specs_delete(self.ptr, ...)
	end
	function META:SetErp(...)
		return CLIB.btSoftBody_Joint_Specs_setErp(self.ptr, ...)
	end
	function META:GetSplit(...)
		return CLIB.btSoftBody_Joint_Specs_getSplit(self.ptr, ...)
	end
	function META:SetSplit(...)
		return CLIB.btSoftBody_Joint_Specs_setSplit(self.ptr, ...)
	end
end
do -- btSoftBodyNodePtrArray
	local META = {}
	library.metatables.btSoftBodyNodePtrArray = META
	META.__index = function(s, k) return META[k] end
	function META:Set(...)
		return CLIB.btSoftBodyNodePtrArray_set(self.ptr, ...)
	end
	function META:At(...)
		return CLIB.btSoftBodyNodePtrArray_at(self.ptr, ...)
	end
end
do -- btCollisionDispatcher
	local META = {}
	library.metatables.btCollisionDispatcher = META
	META.__index = function(s, k) return META[k] end
	function library.CreateCollisionDispatcher(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btCollisionDispatcher_new(...)
		return self
	end
	function META:RegisterCollisionCreateFunc(...)
		return CLIB.btCollisionDispatcher_registerCollisionCreateFunc(self.ptr, ...)
	end
	function META:DefaultNearCallback(...)
		return CLIB.btCollisionDispatcher_defaultNearCallback(self.ptr, ...)
	end
	function META:RegisterClosestPointsCreateFunc(...)
		return CLIB.btCollisionDispatcher_registerClosestPointsCreateFunc(self.ptr, ...)
	end
	function META:GetNearCallback(...)
		return CLIB.btCollisionDispatcher_getNearCallback(self.ptr, ...)
	end
	function META:GetDispatcherFlags(...)
		return CLIB.btCollisionDispatcher_getDispatcherFlags(self.ptr, ...)
	end
	function META:SetDispatcherFlags(...)
		return CLIB.btCollisionDispatcher_setDispatcherFlags(self.ptr, ...)
	end
	function META:SetNearCallback(...)
		return CLIB.btCollisionDispatcher_setNearCallback(self.ptr, ...)
	end
	function META:SetCollisionConfiguration(...)
		return CLIB.btCollisionDispatcher_setCollisionConfiguration(self.ptr, ...)
	end
	function META:GetCollisionConfiguration(...)
		return CLIB.btCollisionDispatcher_getCollisionConfiguration(self.ptr, ...)
	end
end
do -- btNodeOverlapCallback
	local META = {}
	library.metatables.btNodeOverlapCallback = META
	META.__index = function(s, k) return META[k] end
	function META:Delete(...)
		return CLIB.btNodeOverlapCallback_delete(self.ptr, ...)
	end
	function META:ProcessNode(...)
		return CLIB.btNodeOverlapCallback_processNode(self.ptr, ...)
	end
end
do -- btGImpactQuantizedBvh
	local META = {}
	library.metatables.btGImpactQuantizedBvh = META
	META.__index = function(s, k) return META[k] end
	function library.CreateGImpactQuantizedBvh(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btGImpactQuantizedBvh_new(...)
		return self
	end
	function library.CreateGImpactQuantizedBvh2(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btGImpactQuantizedBvh_new2(...)
		return self
	end
	function META:GetEscapeNodeIndex(...)
		return CLIB.btGImpactQuantizedBvh_getEscapeNodeIndex(self.ptr, ...)
	end
	function META:IsLeafNode(...)
		return CLIB.btGImpactQuantizedBvh_isLeafNode(self.ptr, ...)
	end
	function META:BoxQueryTrans(...)
		return CLIB.btGImpactQuantizedBvh_boxQueryTrans(self.ptr, ...)
	end
	function META:GetRightNode(...)
		return CLIB.btGImpactQuantizedBvh_getRightNode(self.ptr, ...)
	end
	function META:HasHierarchy(...)
		return CLIB.btGImpactQuantizedBvh_hasHierarchy(self.ptr, ...)
	end
	function META:GetLeftNode(...)
		return CLIB.btGImpactQuantizedBvh_getLeftNode(self.ptr, ...)
	end
	function META:GetGlobalBox(...)
		return CLIB.btGImpactQuantizedBvh_getGlobalBox(self.ptr, ...)
	end
	function META:Delete(...)
		return CLIB.btGImpactQuantizedBvh_delete(self.ptr, ...)
	end
	function META:GetNodeTriangle(...)
		return CLIB.btGImpactQuantizedBvh_getNodeTriangle(self.ptr, ...)
	end
	function META:SetPrimitiveManager(...)
		return CLIB.btGImpactQuantizedBvh_setPrimitiveManager(self.ptr, ...)
	end
	function META:GetNodeCount(...)
		return CLIB.btGImpactQuantizedBvh_getNodeCount(self.ptr, ...)
	end
	function META:RayQuery(...)
		return CLIB.btGImpactQuantizedBvh_rayQuery(self.ptr, ...)
	end
	function META:SetNodeBound(...)
		return CLIB.btGImpactQuantizedBvh_setNodeBound(self.ptr, ...)
	end
	function META:GetNodeBound(...)
		return CLIB.btGImpactQuantizedBvh_getNodeBound(self.ptr, ...)
	end
	function META:BuildSet(...)
		return CLIB.btGImpactQuantizedBvh_buildSet(self.ptr, ...)
	end
	function META:IsTrimesh(...)
		return CLIB.btGImpactQuantizedBvh_isTrimesh(self.ptr, ...)
	end
	function META:GetPrimitiveManager(...)
		return CLIB.btGImpactQuantizedBvh_getPrimitiveManager(self.ptr, ...)
	end
	function META:BoxQuery(...)
		return CLIB.btGImpactQuantizedBvh_boxQuery(self.ptr, ...)
	end
	function META:Update(...)
		return CLIB.btGImpactQuantizedBvh_update(self.ptr, ...)
	end
	function META:GetNodeData(...)
		return CLIB.btGImpactQuantizedBvh_getNodeData(self.ptr, ...)
	end
end
do -- btMLCPSolverInterface
	local META = {}
	library.metatables.btMLCPSolverInterface = META
	META.__index = function(s, k) return META[k] end
	function META:Delete(...)
		return CLIB.btMLCPSolverInterface_delete(self.ptr, ...)
	end
end
do -- btSoftBody_ImplicitFn
	local META = {}
	library.metatables.btSoftBody_ImplicitFn = META
	META.__index = function(s, k) return META[k] end
	function META:Delete(...)
		return CLIB.btSoftBody_ImplicitFn_delete(self.ptr, ...)
	end
	function META:Eval(...)
		return CLIB.btSoftBody_ImplicitFn_Eval(self.ptr, ...)
	end
end
do -- btMultiBodyJointMotor
	local META = {}
	library.metatables.btMultiBodyJointMotor = META
	META.__index = function(s, k) return META[k] end
	function library.CreateMultiBodyJointMotor(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btMultiBodyJointMotor_new(...)
		return self
	end
	function library.CreateMultiBodyJointMotor2(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btMultiBodyJointMotor_new2(...)
		return self
	end
	function META:SetVelocityTarget(...)
		return CLIB.btMultiBodyJointMotor_setVelocityTarget(self.ptr, ...)
	end
	function META:SetPositionTarget2(...)
		return CLIB.btMultiBodyJointMotor_setPositionTarget2(self.ptr, ...)
	end
	function META:SetVelocityTarget2(...)
		return CLIB.btMultiBodyJointMotor_setVelocityTarget2(self.ptr, ...)
	end
	function META:SetPositionTarget(...)
		return CLIB.btMultiBodyJointMotor_setPositionTarget(self.ptr, ...)
	end
end
do -- btConvexInternalShape
	local META = {}
	library.metatables.btConvexInternalShape = META
	function META:__index(k)
		local v

		v = META[k]
		if v ~= nil then
			return v
		end
		v = library.metatables.btConvexShape.__index(self, k)
		if v ~= nil then
			return v
		end
	end
	function META:SetImplicitShapeDimensions(...)
		return CLIB.btConvexInternalShape_setImplicitShapeDimensions(self.ptr, ...)
	end
	function META:GetImplicitShapeDimensions(...)
		return CLIB.btConvexInternalShape_getImplicitShapeDimensions(self.ptr, ...)
	end
	function META:GetLocalScalingNV(...)
		return CLIB.btConvexInternalShape_getLocalScalingNV(self.ptr, ...)
	end
	function META:GetMarginNV(...)
		return CLIB.btConvexInternalShape_getMarginNV(self.ptr, ...)
	end
	function META:SetSafeMargin(...)
		return CLIB.btConvexInternalShape_setSafeMargin(self.ptr, ...)
	end
	function META:SetSafeMargin2(...)
		return CLIB.btConvexInternalShape_setSafeMargin2(self.ptr, ...)
	end
end
do -- btBroadphaseInterface
	local META = {}
	library.metatables.btBroadphaseInterface = META
	META.__index = function(s, k) return META[k] end
	function META:RayTest2(...)
		return CLIB.btBroadphaseInterface_rayTest2(self.ptr, ...)
	end
	function META:DestroyProxy(...)
		return CLIB.btBroadphaseInterface_destroyProxy(self.ptr, ...)
	end
	function META:CalculateOverlappingPairs(...)
		return CLIB.btBroadphaseInterface_calculateOverlappingPairs(self.ptr, ...)
	end
	function META:AabbTest(...)
		return CLIB.btBroadphaseInterface_aabbTest(self.ptr, ...)
	end
	function META:GetBroadphaseAabb(...)
		return CLIB.btBroadphaseInterface_getBroadphaseAabb(self.ptr, ...)
	end
	function META:GetOverlappingPairCache(...)
		return CLIB.btBroadphaseInterface_getOverlappingPairCache(self.ptr, ...)
	end
	function META:Delete(...)
		return CLIB.btBroadphaseInterface_delete(self.ptr, ...)
	end
	function META:RayTest(...)
		return CLIB.btBroadphaseInterface_rayTest(self.ptr, ...)
	end
	function META:PrintStats(...)
		return CLIB.btBroadphaseInterface_printStats(self.ptr, ...)
	end
	function META:RayTest3(...)
		return CLIB.btBroadphaseInterface_rayTest3(self.ptr, ...)
	end
	function META:CreateProxy(...)
		return CLIB.btBroadphaseInterface_createProxy(self.ptr, ...)
	end
	function META:ResetPool(...)
		return CLIB.btBroadphaseInterface_resetPool(self.ptr, ...)
	end
	function META:GetAabb(...)
		return CLIB.btBroadphaseInterface_getAabb(self.ptr, ...)
	end
	function META:SetAabb(...)
		return CLIB.btBroadphaseInterface_setAabb(self.ptr, ...)
	end
end
do -- btGImpactBvh_get_node
	local META = {}
	library.metatables.btGImpactBvh_get_node = META
	META.__index = function(s, k) return META[k] end
	function META:Pointer(...)
		return CLIB.btGImpactBvh_get_node_pointer(self.ptr, ...)
	end
end
do -- btBulletWorldImporter
	local META = {}
	library.metatables.btBulletWorldImporter = META
	META.__index = function(s, k) return META[k] end
	function library.CreateBulletWorldImporter(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btBulletWorldImporter_new(...)
		return self
	end
	function library.CreateBulletWorldImporter2(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btBulletWorldImporter_new2(...)
		return self
	end
	function META:LoadFile(...)
		return CLIB.btBulletWorldImporter_loadFile(self.ptr, ...)
	end
	function META:LoadFile2(...)
		return CLIB.btBulletWorldImporter_loadFile2(self.ptr, ...)
	end
	function META:LoadFileFromMemory(...)
		return CLIB.btBulletWorldImporter_loadFileFromMemory(self.ptr, ...)
	end
	function META:ConvertAllObjects(...)
		return CLIB.btBulletWorldImporter_convertAllObjects(self.ptr, ...)
	end
	function META:LoadFileFromMemory2(...)
		return CLIB.btBulletWorldImporter_loadFileFromMemory2(self.ptr, ...)
	end
end
do -- btMultiBodyConstraint
	local META = {}
	library.metatables.btMultiBodyConstraint = META
	META.__index = function(s, k) return META[k] end
	function META:JacobianA(...)
		return CLIB.btMultiBodyConstraint_jacobianA(self.ptr, ...)
	end
	function META:InternalSetAppliedImpulse(...)
		return CLIB.btMultiBodyConstraint_internalSetAppliedImpulse(self.ptr, ...)
	end
	function META:GetIslandIdB(...)
		return CLIB.btMultiBodyConstraint_getIslandIdB(self.ptr, ...)
	end
	function META:SetPosition(...)
		return CLIB.btMultiBodyConstraint_setPosition(self.ptr, ...)
	end
	function META:GetMaxAppliedImpulse(...)
		return CLIB.btMultiBodyConstraint_getMaxAppliedImpulse(self.ptr, ...)
	end
	function META:GetNumRows(...)
		return CLIB.btMultiBodyConstraint_getNumRows(self.ptr, ...)
	end
	function META:JacobianB(...)
		return CLIB.btMultiBodyConstraint_jacobianB(self.ptr, ...)
	end
	function META:UpdateJacobianSizes(...)
		return CLIB.btMultiBodyConstraint_updateJacobianSizes(self.ptr, ...)
	end
	function META:GetMultiBodyA(...)
		return CLIB.btMultiBodyConstraint_getMultiBodyA(self.ptr, ...)
	end
	function META:DebugDraw(...)
		return CLIB.btMultiBodyConstraint_debugDraw(self.ptr, ...)
	end
	function META:FinalizeMultiDof(...)
		return CLIB.btMultiBodyConstraint_finalizeMultiDof(self.ptr, ...)
	end
	function META:IsUnilateral(...)
		return CLIB.btMultiBodyConstraint_isUnilateral(self.ptr, ...)
	end
	function META:GetMultiBodyB(...)
		return CLIB.btMultiBodyConstraint_getMultiBodyB(self.ptr, ...)
	end
	function META:GetAppliedImpulse(...)
		return CLIB.btMultiBodyConstraint_getAppliedImpulse(self.ptr, ...)
	end
	function META:GetIslandIdA(...)
		return CLIB.btMultiBodyConstraint_getIslandIdA(self.ptr, ...)
	end
	function META:Delete(...)
		return CLIB.btMultiBodyConstraint_delete(self.ptr, ...)
	end
	function META:AllocateJacobiansMultiDof(...)
		return CLIB.btMultiBodyConstraint_allocateJacobiansMultiDof(self.ptr, ...)
	end
	function META:CreateConstraintRows(...)
		return CLIB.btMultiBodyConstraint_createConstraintRows(self.ptr, ...)
	end
	function META:SetMaxAppliedImpulse(...)
		return CLIB.btMultiBodyConstraint_setMaxAppliedImpulse(self.ptr, ...)
	end
	function META:GetPosition(...)
		return CLIB.btMultiBodyConstraint_getPosition(self.ptr, ...)
	end
end
do -- btCompoundFromGImpact
	local META = {}
	library.metatables.btCompoundFromGImpact = META
	META.__index = function(s, k) return META[k] end
	function META:BtCreateCompoundFromGimpactShape(...)
		return CLIB.btCompoundFromGImpact_btCreateCompoundFromGimpactShape(self.ptr, ...)
	end
end
do -- btUniversalConstraint
	local META = {}
	library.metatables.btUniversalConstraint = META
	META.__index = function(s, k) return META[k] end
	function library.CreateUniversalConstraint(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btUniversalConstraint_new(...)
		return self
	end
	function META:GetAngle1(...)
		return CLIB.btUniversalConstraint_getAngle1(self.ptr, ...)
	end
	function META:GetAnchor2(...)
		return CLIB.btUniversalConstraint_getAnchor2(self.ptr, ...)
	end
	function META:SetUpperLimit(...)
		return CLIB.btUniversalConstraint_setUpperLimit(self.ptr, ...)
	end
	function META:SetLowerLimit(...)
		return CLIB.btUniversalConstraint_setLowerLimit(self.ptr, ...)
	end
	function META:GetAxis2(...)
		return CLIB.btUniversalConstraint_getAxis2(self.ptr, ...)
	end
	function META:GetAnchor(...)
		return CLIB.btUniversalConstraint_getAnchor(self.ptr, ...)
	end
	function META:GetAxis1(...)
		return CLIB.btUniversalConstraint_getAxis1(self.ptr, ...)
	end
	function META:GetAngle2(...)
		return CLIB.btUniversalConstraint_getAngle2(self.ptr, ...)
	end
end
do -- btUniformScalingShape
	local META = {}
	library.metatables.btUniformScalingShape = META
	META.__index = function(s, k) return META[k] end
	function library.CreateUniformScalingShape(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btUniformScalingShape_new(...)
		return self
	end
	function META:GetUniformScalingFactor(...)
		return CLIB.btUniformScalingShape_getUniformScalingFactor(self.ptr, ...)
	end
	function META:GetChildShape(...)
		return CLIB.btUniformScalingShape_getChildShape(self.ptr, ...)
	end
end
do -- btConeTwistConstraint
	local META = {}
	library.metatables.btConeTwistConstraint = META
	META.__index = function(s, k) return META[k] end
	function library.CreateConeTwistConstraint(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btConeTwistConstraint_new2(...)
		return self
	end
	function library.CreateConeTwistConstraint2(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btConeTwistConstraint_new(...)
		return self
	end
	function META:SetFrames(...)
		return CLIB.btConeTwistConstraint_setFrames(self.ptr, ...)
	end
	function META:GetRelaxationFactor(...)
		return CLIB.btConeTwistConstraint_getRelaxationFactor(self.ptr, ...)
	end
	function META:GetSwingSpan1(...)
		return CLIB.btConeTwistConstraint_getSwingSpan1(self.ptr, ...)
	end
	function META:SetFixThresh(...)
		return CLIB.btConeTwistConstraint_setFixThresh(self.ptr, ...)
	end
	function META:SetMotorTargetInConstraintSpace(...)
		return CLIB.btConeTwistConstraint_setMotorTargetInConstraintSpace(self.ptr, ...)
	end
	function META:GetSolveSwingLimit(...)
		return CLIB.btConeTwistConstraint_getSolveSwingLimit(self.ptr, ...)
	end
	function META:IsPastSwingLimit(...)
		return CLIB.btConeTwistConstraint_isPastSwingLimit(self.ptr, ...)
	end
	function META:SetLimit2(...)
		return CLIB.btConeTwistConstraint_setLimit2(self.ptr, ...)
	end
	function META:GetPointForAngle(...)
		return CLIB.btConeTwistConstraint_GetPointForAngle(self.ptr, ...)
	end
	function META:SetMaxMotorImpulse(...)
		return CLIB.btConeTwistConstraint_setMaxMotorImpulse(self.ptr, ...)
	end
	function META:SetMotorTarget(...)
		return CLIB.btConeTwistConstraint_setMotorTarget(self.ptr, ...)
	end
	function META:GetBiasFactor(...)
		return CLIB.btConeTwistConstraint_getBiasFactor(self.ptr, ...)
	end
	function META:IsMaxMotorImpulseNormalized(...)
		return CLIB.btConeTwistConstraint_isMaxMotorImpulseNormalized(self.ptr, ...)
	end
	function META:GetAFrame(...)
		return CLIB.btConeTwistConstraint_getAFrame(self.ptr, ...)
	end
	function META:GetInfo2NonVirtual(...)
		return CLIB.btConeTwistConstraint_getInfo2NonVirtual(self.ptr, ...)
	end
	function META:GetTwistAngle(...)
		return CLIB.btConeTwistConstraint_getTwistAngle(self.ptr, ...)
	end
	function META:GetFixThresh(...)
		return CLIB.btConeTwistConstraint_getFixThresh(self.ptr, ...)
	end
	function META:EnableMotor(...)
		return CLIB.btConeTwistConstraint_enableMotor(self.ptr, ...)
	end
	function META:SetAngularOnly(...)
		return CLIB.btConeTwistConstraint_setAngularOnly(self.ptr, ...)
	end
	function META:CalcAngleInfo2(...)
		return CLIB.btConeTwistConstraint_calcAngleInfo2(self.ptr, ...)
	end
	function META:GetFrameOffsetB(...)
		return CLIB.btConeTwistConstraint_getFrameOffsetB(self.ptr, ...)
	end
	function META:UpdateRHS(...)
		return CLIB.btConeTwistConstraint_updateRHS(self.ptr, ...)
	end
	function META:GetFrameOffsetA(...)
		return CLIB.btConeTwistConstraint_getFrameOffsetA(self.ptr, ...)
	end
	function META:SetDamping(...)
		return CLIB.btConeTwistConstraint_setDamping(self.ptr, ...)
	end
	function META:IsMotorEnabled(...)
		return CLIB.btConeTwistConstraint_isMotorEnabled(self.ptr, ...)
	end
	function META:CalcAngleInfo(...)
		return CLIB.btConeTwistConstraint_calcAngleInfo(self.ptr, ...)
	end
	function META:SetMaxMotorImpulseNormalized(...)
		return CLIB.btConeTwistConstraint_setMaxMotorImpulseNormalized(self.ptr, ...)
	end
	function META:SetLimit(...)
		return CLIB.btConeTwistConstraint_setLimit(self.ptr, ...)
	end
	function META:GetTwistSpan(...)
		return CLIB.btConeTwistConstraint_getTwistSpan(self.ptr, ...)
	end
	function META:GetInfo1NonVirtual(...)
		return CLIB.btConeTwistConstraint_getInfo1NonVirtual(self.ptr, ...)
	end
	function META:GetSwingSpan2(...)
		return CLIB.btConeTwistConstraint_getSwingSpan2(self.ptr, ...)
	end
	function META:GetMotorTarget(...)
		return CLIB.btConeTwistConstraint_getMotorTarget(self.ptr, ...)
	end
	function META:GetDamping(...)
		return CLIB.btConeTwistConstraint_getDamping(self.ptr, ...)
	end
	function META:GetLimit(...)
		return CLIB.btConeTwistConstraint_getLimit(self.ptr, ...)
	end
	function META:GetBFrame(...)
		return CLIB.btConeTwistConstraint_getBFrame(self.ptr, ...)
	end
	function META:GetAngularOnly(...)
		return CLIB.btConeTwistConstraint_getAngularOnly(self.ptr, ...)
	end
	function META:GetFlags(...)
		return CLIB.btConeTwistConstraint_getFlags(self.ptr, ...)
	end
	function META:GetMaxMotorImpulse(...)
		return CLIB.btConeTwistConstraint_getMaxMotorImpulse(self.ptr, ...)
	end
	function META:GetLimitSoftness(...)
		return CLIB.btConeTwistConstraint_getLimitSoftness(self.ptr, ...)
	end
	function META:GetSolveTwistLimit(...)
		return CLIB.btConeTwistConstraint_getSolveTwistLimit(self.ptr, ...)
	end
	function META:GetTwistLimitSign(...)
		return CLIB.btConeTwistConstraint_getTwistLimitSign(self.ptr, ...)
	end
end
do -- btTetrahedronShapeEx
	local META = {}
	library.metatables.btTetrahedronShapeEx = META
	META.__index = function(s, k) return META[k] end
	function library.CreateTetrahedronShapeEx(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btTetrahedronShapeEx_new(...)
		return self
	end
	function META:SetVertices(...)
		return CLIB.btTetrahedronShapeEx_setVertices(self.ptr, ...)
	end
end
do -- btMotionStateWrapper
	local META = {}
	library.metatables.btMotionStateWrapper = META
	META.__index = function(s, k) return META[k] end
	function library.CreateMotionStateWrapper(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btMotionStateWrapper_new(...)
		return self
	end
end
do -- btDefaultMotionState
	local META = {}
	library.metatables.btDefaultMotionState = META
	META.__index = function(s, k) return META[k] end
	function library.CreateDefaultMotionState(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btDefaultMotionState_new2(...)
		return self
	end
	function library.CreateDefaultMotionState2(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btDefaultMotionState_new(...)
		return self
	end
	function library.CreateDefaultMotionState3(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btDefaultMotionState_new3(...)
		return self
	end
	function META:SetCenterOfMassOffset(...)
		return CLIB.btDefaultMotionState_setCenterOfMassOffset(self.ptr, ...)
	end
	function META:SetGraphicsWorldTrans(...)
		return CLIB.btDefaultMotionState_setGraphicsWorldTrans(self.ptr, ...)
	end
	function META:GetCenterOfMassOffset(...)
		return CLIB.btDefaultMotionState_getCenterOfMassOffset(self.ptr, ...)
	end
	function META:SetUserPointer(...)
		return CLIB.btDefaultMotionState_setUserPointer(self.ptr, ...)
	end
	function META:GetGraphicsWorldTrans(...)
		return CLIB.btDefaultMotionState_getGraphicsWorldTrans(self.ptr, ...)
	end
	function META:GetUserPointer(...)
		return CLIB.btDefaultMotionState_getUserPointer(self.ptr, ...)
	end
	function META:SetStartWorldTrans(...)
		return CLIB.btDefaultMotionState_setStartWorldTrans(self.ptr, ...)
	end
	function META:GetStartWorldTrans(...)
		return CLIB.btDefaultMotionState_getStartWorldTrans(self.ptr, ...)
	end
end
do -- btPolarDecomposition
	local META = {}
	library.metatables.btPolarDecomposition = META
	META.__index = function(s, k) return META[k] end
	function library.CreatePolarDecomposition(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btPolarDecomposition_new(...)
		return self
	end
	function META:MaxIterations(...)
		return CLIB.btPolarDecomposition_maxIterations(self.ptr, ...)
	end
	function META:Delete(...)
		return CLIB.btPolarDecomposition_delete(self.ptr, ...)
	end
	function META:Decompose(...)
		return CLIB.btPolarDecomposition_decompose(self.ptr, ...)
	end
end
do -- btMaterialProperties
	local META = {}
	library.metatables.btMaterialProperties = META
	META.__index = function(s, k) return META[k] end
	function library.CreateMaterialProperties(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btMaterialProperties_new(...)
		return self
	end
	function META:GetTriangleMaterialsBase(...)
		return CLIB.btMaterialProperties_getTriangleMaterialsBase(self.ptr, ...)
	end
	function META:SetTriangleMaterialsBase(...)
		return CLIB.btMaterialProperties_setTriangleMaterialsBase(self.ptr, ...)
	end
	function META:GetMaterialType(...)
		return CLIB.btMaterialProperties_getMaterialType(self.ptr, ...)
	end
	function META:SetMaterialType(...)
		return CLIB.btMaterialProperties_setMaterialType(self.ptr, ...)
	end
	function META:GetTriangleMaterialStride(...)
		return CLIB.btMaterialProperties_getTriangleMaterialStride(self.ptr, ...)
	end
	function META:Delete(...)
		return CLIB.btMaterialProperties_delete(self.ptr, ...)
	end
	function META:SetTriangleType(...)
		return CLIB.btMaterialProperties_setTriangleType(self.ptr, ...)
	end
	function META:SetNumTriangles(...)
		return CLIB.btMaterialProperties_setNumTriangles(self.ptr, ...)
	end
	function META:SetNumMaterials(...)
		return CLIB.btMaterialProperties_setNumMaterials(self.ptr, ...)
	end
	function META:SetMaterialStride(...)
		return CLIB.btMaterialProperties_setMaterialStride(self.ptr, ...)
	end
	function META:SetMaterialBase(...)
		return CLIB.btMaterialProperties_setMaterialBase(self.ptr, ...)
	end
	function META:GetTriangleType(...)
		return CLIB.btMaterialProperties_getTriangleType(self.ptr, ...)
	end
	function META:GetMaterialStride(...)
		return CLIB.btMaterialProperties_getMaterialStride(self.ptr, ...)
	end
	function META:GetNumTriangles(...)
		return CLIB.btMaterialProperties_getNumTriangles(self.ptr, ...)
	end
	function META:GetNumMaterials(...)
		return CLIB.btMaterialProperties_getNumMaterials(self.ptr, ...)
	end
	function META:SetTriangleMaterialStride(...)
		return CLIB.btMaterialProperties_setTriangleMaterialStride(self.ptr, ...)
	end
	function META:GetMaterialBase(...)
		return CLIB.btMaterialProperties_getMaterialBase(self.ptr, ...)
	end
end
do -- btPersistentManifold
	local META = {}
	library.metatables.btPersistentManifold = META
	META.__index = function(s, k) return META[k] end
	function library.CreatePersistentManifold(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btPersistentManifold_new2(...)
		return self
	end
	function library.CreatePersistentManifold2(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btPersistentManifold_new(...)
		return self
	end
	function META:GetIndex1a(...)
		return CLIB.btPersistentManifold_getIndex1a(self.ptr, ...)
	end
	function META:GetContactBreakingThreshold(...)
		return CLIB.btPersistentManifold_getContactBreakingThreshold(self.ptr, ...)
	end
	function META:ReplaceContactPoint(...)
		return CLIB.btPersistentManifold_replaceContactPoint(self.ptr, ...)
	end
	function META:Delete(...)
		return CLIB.btPersistentManifold_delete(self.ptr, ...)
	end
	function META:GetCacheEntry(...)
		return CLIB.btPersistentManifold_getCacheEntry(self.ptr, ...)
	end
	function META:GetCompanionIdB(...)
		return CLIB.btPersistentManifold_getCompanionIdB(self.ptr, ...)
	end
	function META:GetBody0(...)
		return CLIB.btPersistentManifold_getBody0(self.ptr, ...)
	end
	function META:GetCompanionIdA(...)
		return CLIB.btPersistentManifold_getCompanionIdA(self.ptr, ...)
	end
	function META:GetContactProcessingThreshold(...)
		return CLIB.btPersistentManifold_getContactProcessingThreshold(self.ptr, ...)
	end
	function META:ValidContactDistance(...)
		return CLIB.btPersistentManifold_validContactDistance(self.ptr, ...)
	end
	function META:GetNumContacts(...)
		return CLIB.btPersistentManifold_getNumContacts(self.ptr, ...)
	end
	function META:SetNumContacts(...)
		return CLIB.btPersistentManifold_setNumContacts(self.ptr, ...)
	end
	function META:SetIndex1a(...)
		return CLIB.btPersistentManifold_setIndex1a(self.ptr, ...)
	end
	function META:SetContactProcessingThreshold(...)
		return CLIB.btPersistentManifold_setContactProcessingThreshold(self.ptr, ...)
	end
	function META:SetContactBreakingThreshold(...)
		return CLIB.btPersistentManifold_setContactBreakingThreshold(self.ptr, ...)
	end
	function META:SetCompanionIdB(...)
		return CLIB.btPersistentManifold_setCompanionIdB(self.ptr, ...)
	end
	function META:SetCompanionIdA(...)
		return CLIB.btPersistentManifold_setCompanionIdA(self.ptr, ...)
	end
	function META:GetContactPoint(...)
		return CLIB.btPersistentManifold_getContactPoint(self.ptr, ...)
	end
	function META:GetBody1(...)
		return CLIB.btPersistentManifold_getBody1(self.ptr, ...)
	end
	function META:ClearUserCache(...)
		return CLIB.btPersistentManifold_clearUserCache(self.ptr, ...)
	end
	function META:ClearManifold(...)
		return CLIB.btPersistentManifold_clearManifold(self.ptr, ...)
	end
	function META:RemoveContactPoint(...)
		return CLIB.btPersistentManifold_removeContactPoint(self.ptr, ...)
	end
	function META:RefreshContactPoints(...)
		return CLIB.btPersistentManifold_refreshContactPoints(self.ptr, ...)
	end
	function META:AddManifoldPoint(...)
		return CLIB.btPersistentManifold_addManifoldPoint(self.ptr, ...)
	end
	function META:SetBodies(...)
		return CLIB.btPersistentManifold_setBodies(self.ptr, ...)
	end
end
do -- btCollisionAlgorithm
	local META = {}
	library.metatables.btCollisionAlgorithm = META
	META.__index = function(s, k) return META[k] end
	function META:ProcessCollision(...)
		return CLIB.btCollisionAlgorithm_processCollision(self.ptr, ...)
	end
	function META:CalculateTimeOfImpact(...)
		return CLIB.btCollisionAlgorithm_calculateTimeOfImpact(self.ptr, ...)
	end
	function META:Delete(...)
		return CLIB.btCollisionAlgorithm_delete(self.ptr, ...)
	end
	function META:GetAllContactManifolds(...)
		return CLIB.btCollisionAlgorithm_getAllContactManifolds(self.ptr, ...)
	end
end
do -- btCompoundShapeChild
	local META = {}
	library.metatables.btCompoundShapeChild = META
	META.__index = function(s, k) return META[k] end
	function META:GetTransform(...)
		return CLIB.btCompoundShapeChild_getTransform(self.ptr, ...)
	end
	function META:GetNode(...)
		return CLIB.btCompoundShapeChild_getNode(self.ptr, ...)
	end
	function META:SetNode(...)
		return CLIB.btCompoundShapeChild_setNode(self.ptr, ...)
	end
	function META:SetChildShapeType(...)
		return CLIB.btCompoundShapeChild_setChildShapeType(self.ptr, ...)
	end
	function META:Delete(...)
		return CLIB.btCompoundShapeChild_delete(self.ptr, ...)
	end
	function META:GetChildShape(...)
		return CLIB.btCompoundShapeChild_getChildShape(self.ptr, ...)
	end
	function META:SetChildShape(...)
		return CLIB.btCompoundShapeChild_setChildShape(self.ptr, ...)
	end
	function META:SetChildMargin(...)
		return CLIB.btCompoundShapeChild_setChildMargin(self.ptr, ...)
	end
	function META:GetChildMargin(...)
		return CLIB.btCompoundShapeChild_getChildMargin(self.ptr, ...)
	end
	function META:SetTransform(...)
		return CLIB.btCompoundShapeChild_setTransform(self.ptr, ...)
	end
	function META:GetChildShapeType(...)
		return CLIB.btCompoundShapeChild_getChildShapeType(self.ptr, ...)
	end
end
do -- btSoftBody_RContact
	local META = {}
	library.metatables.btSoftBody_RContact = META
	META.__index = function(s, k) return META[k] end
	function library.CreateSoftBody_RContact(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btSoftBody_RContact_new(...)
		return self
	end
	function META:GetCti(...)
		return CLIB.btSoftBody_RContact_getCti(self.ptr, ...)
	end
	function META:GetC4(...)
		return CLIB.btSoftBody_RContact_getC4(self.ptr, ...)
	end
	function META:SetC4(...)
		return CLIB.btSoftBody_RContact_setC4(self.ptr, ...)
	end
	function META:GetC0(...)
		return CLIB.btSoftBody_RContact_getC0(self.ptr, ...)
	end
	function META:GetC1(...)
		return CLIB.btSoftBody_RContact_getC1(self.ptr, ...)
	end
	function META:SetNode(...)
		return CLIB.btSoftBody_RContact_setNode(self.ptr, ...)
	end
	function META:SetC3(...)
		return CLIB.btSoftBody_RContact_setC3(self.ptr, ...)
	end
	function META:SetC2(...)
		return CLIB.btSoftBody_RContact_setC2(self.ptr, ...)
	end
	function META:SetC1(...)
		return CLIB.btSoftBody_RContact_setC1(self.ptr, ...)
	end
	function META:SetC0(...)
		return CLIB.btSoftBody_RContact_setC0(self.ptr, ...)
	end
	function META:GetNode(...)
		return CLIB.btSoftBody_RContact_getNode(self.ptr, ...)
	end
	function META:GetC3(...)
		return CLIB.btSoftBody_RContact_getC3(self.ptr, ...)
	end
	function META:GetC2(...)
		return CLIB.btSoftBody_RContact_getC2(self.ptr, ...)
	end
	function META:Delete(...)
		return CLIB.btSoftBody_RContact_delete(self.ptr, ...)
	end
end
do -- btSoftBody_SContact
	local META = {}
	library.metatables.btSoftBody_SContact = META
	META.__index = function(s, k) return META[k] end
	function library.CreateSoftBody_SContact(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btSoftBody_SContact_new(...)
		return self
	end
	function META:GetFace(...)
		return CLIB.btSoftBody_SContact_getFace(self.ptr, ...)
	end
	function META:SetWeights(...)
		return CLIB.btSoftBody_SContact_setWeights(self.ptr, ...)
	end
	function META:SetNormal(...)
		return CLIB.btSoftBody_SContact_setNormal(self.ptr, ...)
	end
	function META:SetNode(...)
		return CLIB.btSoftBody_SContact_setNode(self.ptr, ...)
	end
	function META:SetMargin(...)
		return CLIB.btSoftBody_SContact_setMargin(self.ptr, ...)
	end
	function META:SetFriction(...)
		return CLIB.btSoftBody_SContact_setFriction(self.ptr, ...)
	end
	function META:SetFace(...)
		return CLIB.btSoftBody_SContact_setFace(self.ptr, ...)
	end
	function META:GetNormal(...)
		return CLIB.btSoftBody_SContact_getNormal(self.ptr, ...)
	end
	function META:GetNode(...)
		return CLIB.btSoftBody_SContact_getNode(self.ptr, ...)
	end
	function META:GetFriction(...)
		return CLIB.btSoftBody_SContact_getFriction(self.ptr, ...)
	end
	function META:GetCfm(...)
		return CLIB.btSoftBody_SContact_getCfm(self.ptr, ...)
	end
	function META:Delete(...)
		return CLIB.btSoftBody_SContact_delete(self.ptr, ...)
	end
	function META:GetWeights(...)
		return CLIB.btSoftBody_SContact_getWeights(self.ptr, ...)
	end
	function META:GetMargin(...)
		return CLIB.btSoftBody_SContact_getMargin(self.ptr, ...)
	end
end
do -- btTriangleMeshShape
	local META = {}
	library.metatables.btTriangleMeshShape = META
	META.__index = function(s, k) return META[k] end
	function META:GetLocalAabbMax(...)
		return CLIB.btTriangleMeshShape_getLocalAabbMax(self.ptr, ...)
	end
	function META:LocalGetSupportingVertexWithoutMargin(...)
		return CLIB.btTriangleMeshShape_localGetSupportingVertexWithoutMargin(self.ptr, ...)
	end
	function META:GetMeshInterface(...)
		return CLIB.btTriangleMeshShape_getMeshInterface(self.ptr, ...)
	end
	function META:GetLocalAabbMin(...)
		return CLIB.btTriangleMeshShape_getLocalAabbMin(self.ptr, ...)
	end
	function META:RecalcLocalAabb(...)
		return CLIB.btTriangleMeshShape_recalcLocalAabb(self.ptr, ...)
	end
	function META:LocalGetSupportingVertex(...)
		return CLIB.btTriangleMeshShape_localGetSupportingVertex(self.ptr, ...)
	end
end
do -- btMinkowskiSumShape
	local META = {}
	library.metatables.btMinkowskiSumShape = META
	META.__index = function(s, k) return META[k] end
	function library.CreateMinkowskiSumShape(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btMinkowskiSumShape_new(...)
		return self
	end
	function META:GetShapeA(...)
		return CLIB.btMinkowskiSumShape_getShapeA(self.ptr, ...)
	end
	function META:GetTransformB(...)
		return CLIB.btMinkowskiSumShape_GetTransformB(self.ptr, ...)
	end
	function META:SetTransformB(...)
		return CLIB.btMinkowskiSumShape_setTransformB(self.ptr, ...)
	end
	function META:SetTransformA(...)
		return CLIB.btMinkowskiSumShape_setTransformA(self.ptr, ...)
	end
	function META:GetShapeB(...)
		return CLIB.btMinkowskiSumShape_getShapeB(self.ptr, ...)
	end
	function META:GetTransformA(...)
		return CLIB.btMinkowskiSumShape_getTransformA(self.ptr, ...)
	end
end
do -- btSerializerWrapper
	local META = {}
	library.metatables.btSerializerWrapper = META
	META.__index = function(s, k) return META[k] end
	function library.CreateSerializerWrapper(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btSerializerWrapper_new(...)
		return self
	end
end
do -- btConstraintSetting
	local META = {}
	library.metatables.btConstraintSetting = META
	META.__index = function(s, k) return META[k] end
	function library.CreateConstraintSetting(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btConstraintSetting_new(...)
		return self
	end
	function META:SetImpulseClamp(...)
		return CLIB.btConstraintSetting_setImpulseClamp(self.ptr, ...)
	end
	function META:GetImpulseClamp(...)
		return CLIB.btConstraintSetting_getImpulseClamp(self.ptr, ...)
	end
	function META:Delete(...)
		return CLIB.btConstraintSetting_delete(self.ptr, ...)
	end
	function META:SetTau(...)
		return CLIB.btConstraintSetting_setTau(self.ptr, ...)
	end
	function META:SetDamping(...)
		return CLIB.btConstraintSetting_setDamping(self.ptr, ...)
	end
	function META:GetTau(...)
		return CLIB.btConstraintSetting_getTau(self.ptr, ...)
	end
	function META:GetDamping(...)
		return CLIB.btConstraintSetting_getDamping(self.ptr, ...)
	end
end
do -- btGhostPairCallback
	local META = {}
	library.metatables.btGhostPairCallback = META
	META.__index = function(s, k) return META[k] end
	function library.CreateGhostPairCallback(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btGhostPairCallback_new(...)
		return self
	end
end
do -- btIDebugDrawWrapper
	local META = {}
	library.metatables.btIDebugDrawWrapper = META
	META.__index = function(s, k) return META[k] end
	function library.CreateIDebugDrawWrapper(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btIDebugDrawWrapper_new(...)
		return self
	end
	function META:GetGCHandle(...)
		return CLIB.btIDebugDrawWrapper_getGCHandle(self.ptr, ...)
	end
end
do -- btSoftBodyWorldInfo
	local META = {}
	library.metatables.btSoftBodyWorldInfo = META
	META.__index = function(s, k) return META[k] end
	function library.CreateSoftBodyWorldInfo(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btSoftBodyWorldInfo_new(...)
		return self
	end
	function META:Delete(...)
		return CLIB.btSoftBodyWorldInfo_delete(self.ptr, ...)
	end
	function META:SetGravity(...)
		return CLIB.btSoftBodyWorldInfo_setGravity(self.ptr, ...)
	end
	function META:SetMaxDisplacement(...)
		return CLIB.btSoftBodyWorldInfo_setMaxDisplacement(self.ptr, ...)
	end
	function META:SetDispatcher(...)
		return CLIB.btSoftBodyWorldInfo_setDispatcher(self.ptr, ...)
	end
	function META:SetBroadphase(...)
		return CLIB.btSoftBodyWorldInfo_setBroadphase(self.ptr, ...)
	end
	function META:GetMaxDisplacement(...)
		return CLIB.btSoftBodyWorldInfo_getMaxDisplacement(self.ptr, ...)
	end
	function META:GetGravity(...)
		return CLIB.btSoftBodyWorldInfo_getGravity(self.ptr, ...)
	end
	function META:GetDispatcher(...)
		return CLIB.btSoftBodyWorldInfo_getDispatcher(self.ptr, ...)
	end
	function META:GetSparsesdf(...)
		return CLIB.btSoftBodyWorldInfo_getSparsesdf(self.ptr, ...)
	end
	function META:GetBroadphase(...)
		return CLIB.btSoftBodyWorldInfo_getBroadphase(self.ptr, ...)
	end
end
do -- btPairSet_push_pair
	local META = {}
	library.metatables.btPairSet_push_pair = META
	META.__index = function(s, k) return META[k] end
	function META:BtPairSet_push_pair(...)
		return CLIB.btPairSet_push_pair(self.ptr, ...)
	end
	function META:Inv(...)
		return CLIB.btPairSet_push_pair_inv(self.ptr, ...)
	end
end
do -- btSoftBody_Material
	local META = {}
	library.metatables.btSoftBody_Material = META
	META.__index = function(s, k) return META[k] end
	function META:GetKAST(...)
		return CLIB.btSoftBody_Material_getKAST(self.ptr, ...)
	end
	function META:GetKLST(...)
		return CLIB.btSoftBody_Material_getKLST(self.ptr, ...)
	end
	function META:SetKVST(...)
		return CLIB.btSoftBody_Material_setKVST(self.ptr, ...)
	end
	function META:SetKLST(...)
		return CLIB.btSoftBody_Material_setKLST(self.ptr, ...)
	end
	function META:SetKAST(...)
		return CLIB.btSoftBody_Material_setKAST(self.ptr, ...)
	end
	function META:SetFlags(...)
		return CLIB.btSoftBody_Material_setFlags(self.ptr, ...)
	end
	function META:GetKVST(...)
		return CLIB.btSoftBody_Material_getKVST(self.ptr, ...)
	end
	function META:GetFlags(...)
		return CLIB.btSoftBody_Material_getFlags(self.ptr, ...)
	end
end
do -- btDbvtNodePtr_array
	local META = {}
	library.metatables.btDbvtNodePtr_array = META
	META.__index = function(s, k) return META[k] end
	function META:At(...)
		return CLIB.btDbvtNodePtr_array_at(self.ptr, ...)
	end
end
do -- btDefaultSerializer
	local META = {}
	library.metatables.btDefaultSerializer = META
	META.__index = function(s, k) return META[k] end
	function library.CreateDefaultSerializer(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btDefaultSerializer_new(...)
		return self
	end
	function library.CreateDefaultSerializer2(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btDefaultSerializer_new2(...)
		return self
	end
	function META:InternalAlloc(...)
		return CLIB.btDefaultSerializer_internalAlloc(self.ptr, ...)
	end
	function META:WriteHeader(...)
		return CLIB.btDefaultSerializer_writeHeader(self.ptr, ...)
	end
end
do -- btContactConstraint
	local META = {}
	library.metatables.btContactConstraint = META
	META.__index = function(s, k) return META[k] end
	function META:SetContactManifold(...)
		return CLIB.btContactConstraint_setContactManifold(self.ptr, ...)
	end
	function META:GetContactManifold(...)
		return CLIB.btContactConstraint_getContactManifold(self.ptr, ...)
	end
end
do -- btSoftBody_sRayCast
	local META = {}
	library.metatables.btSoftBody_sRayCast = META
	META.__index = function(s, k) return META[k] end
	function library.CreateSoftBody_sRayCast(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btSoftBody_sRayCast_new(...)
		return self
	end
	function META:GetFeature(...)
		return CLIB.btSoftBody_sRayCast_getFeature(self.ptr, ...)
	end
	function META:GetBody(...)
		return CLIB.btSoftBody_sRayCast_getBody(self.ptr, ...)
	end
	function META:Delete(...)
		return CLIB.btSoftBody_sRayCast_delete(self.ptr, ...)
	end
	function META:SetIndex(...)
		return CLIB.btSoftBody_sRayCast_setIndex(self.ptr, ...)
	end
	function META:SetFraction(...)
		return CLIB.btSoftBody_sRayCast_setFraction(self.ptr, ...)
	end
	function META:SetFeature(...)
		return CLIB.btSoftBody_sRayCast_setFeature(self.ptr, ...)
	end
	function META:SetBody(...)
		return CLIB.btSoftBody_sRayCast_setBody(self.ptr, ...)
	end
	function META:GetIndex(...)
		return CLIB.btSoftBody_sRayCast_getIndex(self.ptr, ...)
	end
	function META:GetFraction(...)
		return CLIB.btSoftBody_sRayCast_getFraction(self.ptr, ...)
	end
end
do -- btContactSolverInfo
	local META = {}
	library.metatables.btContactSolverInfo = META
	META.__index = function(s, k) return META[k] end
	function library.CreateContactSolverInfo(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btContactSolverInfo_new(...)
		return self
	end
end
do -- btPrimitiveTriangle
	local META = {}
	library.metatables.btPrimitiveTriangle = META
	META.__index = function(s, k) return META[k] end
	function library.CreatePrimitiveTriangle(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btPrimitiveTriangle_new(...)
		return self
	end
	function META:Delete(...)
		return CLIB.btPrimitiveTriangle_delete(self.ptr, ...)
	end
	function META:SetPlane(...)
		return CLIB.btPrimitiveTriangle_setPlane(self.ptr, ...)
	end
	function META:SetMargin(...)
		return CLIB.btPrimitiveTriangle_setMargin(self.ptr, ...)
	end
	function META:SetDummy(...)
		return CLIB.btPrimitiveTriangle_setDummy(self.ptr, ...)
	end
	function META:GetVertices(...)
		return CLIB.btPrimitiveTriangle_getVertices(self.ptr, ...)
	end
	function META:GetPlane(...)
		return CLIB.btPrimitiveTriangle_getPlane(self.ptr, ...)
	end
	function META:GetMargin(...)
		return CLIB.btPrimitiveTriangle_getMargin(self.ptr, ...)
	end
	function META:GetDummy(...)
		return CLIB.btPrimitiveTriangle_getDummy(self.ptr, ...)
	end
	function META:BuildTriPlane(...)
		return CLIB.btPrimitiveTriangle_buildTriPlane(self.ptr, ...)
	end
	function META:ApplyTransform(...)
		return CLIB.btPrimitiveTriangle_applyTransform(self.ptr, ...)
	end
end
do -- btSoftBody_Element
	local META = {}
	library.metatables.btSoftBody_Element = META
	META.__index = function(s, k) return META[k] end
	function META:GetTag(...)
		return CLIB.btSoftBody_Element_getTag(self.ptr, ...)
	end
	function META:Delete(...)
		return CLIB.btSoftBody_Element_delete(self.ptr, ...)
	end
	function META:SetTag(...)
		return CLIB.btSoftBody_Element_setTag(self.ptr, ...)
	end
end
do -- btQuantizedBvhNode
	local META = {}
	library.metatables.btQuantizedBvhNode = META
	META.__index = function(s, k) return META[k] end
	function library.CreateQuantizedBvhNode(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btQuantizedBvhNode_new(...)
		return self
	end
	function META:GetEscapeIndex(...)
		return CLIB.btQuantizedBvhNode_getEscapeIndex(self.ptr, ...)
	end
	function META:GetQuantizedAabbMin(...)
		return CLIB.btQuantizedBvhNode_getQuantizedAabbMin(self.ptr, ...)
	end
	function META:Delete(...)
		return CLIB.btQuantizedBvhNode_delete(self.ptr, ...)
	end
	function META:SetEscapeIndexOrTriangleIndex(...)
		return CLIB.btQuantizedBvhNode_setEscapeIndexOrTriangleIndex(self.ptr, ...)
	end
	function META:IsLeafNode(...)
		return CLIB.btQuantizedBvhNode_isLeafNode(self.ptr, ...)
	end
	function META:GetTriangleIndex(...)
		return CLIB.btQuantizedBvhNode_getTriangleIndex(self.ptr, ...)
	end
	function META:GetPartId(...)
		return CLIB.btQuantizedBvhNode_getPartId(self.ptr, ...)
	end
	function META:GetEscapeIndexOrTriangleIndex(...)
		return CLIB.btQuantizedBvhNode_getEscapeIndexOrTriangleIndex(self.ptr, ...)
	end
	function META:GetQuantizedAabbMax(...)
		return CLIB.btQuantizedBvhNode_getQuantizedAabbMax(self.ptr, ...)
	end
end
do -- btSoftBody_Cluster
	local META = {}
	library.metatables.btSoftBody_Cluster = META
	META.__index = function(s, k) return META[k] end
	function META:SetContainsAnchor(...)
		return CLIB.btSoftBody_Cluster_setContainsAnchor(self.ptr, ...)
	end
	function META:GetIdmass(...)
		return CLIB.btSoftBody_Cluster_getIdmass(self.ptr, ...)
	end
	function META:SetCollide(...)
		return CLIB.btSoftBody_Cluster_setCollide(self.ptr, ...)
	end
	function META:GetDimpulses(...)
		return CLIB.btSoftBody_Cluster_getDimpulses(self.ptr, ...)
	end
	function META:SetCom(...)
		return CLIB.btSoftBody_Cluster_setCom(self.ptr, ...)
	end
	function META:GetMatching(...)
		return CLIB.btSoftBody_Cluster_getMatching(self.ptr, ...)
	end
	function META:SetNdamping(...)
		return CLIB.btSoftBody_Cluster_setNdamping(self.ptr, ...)
	end
	function META:SetFramexform(...)
		return CLIB.btSoftBody_Cluster_setFramexform(self.ptr, ...)
	end
	function META:GetLocii(...)
		return CLIB.btSoftBody_Cluster_getLocii(self.ptr, ...)
	end
	function META:GetMasses(...)
		return CLIB.btSoftBody_Cluster_getMasses(self.ptr, ...)
	end
	function META:GetNodes(...)
		return CLIB.btSoftBody_Cluster_getNodes(self.ptr, ...)
	end
	function META:SetSelfCollisionImpulseFactor(...)
		return CLIB.btSoftBody_Cluster_setSelfCollisionImpulseFactor(self.ptr, ...)
	end
	function META:SetNvimpulses(...)
		return CLIB.btSoftBody_Cluster_setNvimpulses(self.ptr, ...)
	end
	function META:SetNdimpulses(...)
		return CLIB.btSoftBody_Cluster_setNdimpulses(self.ptr, ...)
	end
	function META:SetMaxSelfCollisionImpulse(...)
		return CLIB.btSoftBody_Cluster_setMaxSelfCollisionImpulse(self.ptr, ...)
	end
	function META:SetMatching(...)
		return CLIB.btSoftBody_Cluster_setMatching(self.ptr, ...)
	end
	function META:SetLv(...)
		return CLIB.btSoftBody_Cluster_setLv(self.ptr, ...)
	end
	function META:SetLocii(...)
		return CLIB.btSoftBody_Cluster_setLocii(self.ptr, ...)
	end
	function META:SetLeaf(...)
		return CLIB.btSoftBody_Cluster_setLeaf(self.ptr, ...)
	end
	function META:SetLdamping(...)
		return CLIB.btSoftBody_Cluster_setLdamping(self.ptr, ...)
	end
	function META:SetInvwi(...)
		return CLIB.btSoftBody_Cluster_setInvwi(self.ptr, ...)
	end
	function META:SetImass(...)
		return CLIB.btSoftBody_Cluster_setImass(self.ptr, ...)
	end
	function META:SetIdmass(...)
		return CLIB.btSoftBody_Cluster_setIdmass(self.ptr, ...)
	end
	function META:SetClusterIndex(...)
		return CLIB.btSoftBody_Cluster_setClusterIndex(self.ptr, ...)
	end
	function META:SetAv(...)
		return CLIB.btSoftBody_Cluster_setAv(self.ptr, ...)
	end
	function META:SetAdamping(...)
		return CLIB.btSoftBody_Cluster_setAdamping(self.ptr, ...)
	end
	function META:GetVimpulses(...)
		return CLIB.btSoftBody_Cluster_getVimpulses(self.ptr, ...)
	end
	function META:GetNvimpulses(...)
		return CLIB.btSoftBody_Cluster_getNvimpulses(self.ptr, ...)
	end
	function META:GetNdamping(...)
		return CLIB.btSoftBody_Cluster_getNdamping(self.ptr, ...)
	end
	function META:GetMaxSelfCollisionImpulse(...)
		return CLIB.btSoftBody_Cluster_getMaxSelfCollisionImpulse(self.ptr, ...)
	end
	function META:GetLv(...)
		return CLIB.btSoftBody_Cluster_getLv(self.ptr, ...)
	end
	function META:GetLdamping(...)
		return CLIB.btSoftBody_Cluster_getLdamping(self.ptr, ...)
	end
	function META:GetInvwi(...)
		return CLIB.btSoftBody_Cluster_getInvwi(self.ptr, ...)
	end
	function META:GetImass(...)
		return CLIB.btSoftBody_Cluster_getImass(self.ptr, ...)
	end
	function META:GetFramexform(...)
		return CLIB.btSoftBody_Cluster_getFramexform(self.ptr, ...)
	end
	function META:GetCollide(...)
		return CLIB.btSoftBody_Cluster_getCollide(self.ptr, ...)
	end
	function META:GetClusterIndex(...)
		return CLIB.btSoftBody_Cluster_getClusterIndex(self.ptr, ...)
	end
	function META:GetAdamping(...)
		return CLIB.btSoftBody_Cluster_getAdamping(self.ptr, ...)
	end
	function META:GetNdimpulses(...)
		return CLIB.btSoftBody_Cluster_getNdimpulses(self.ptr, ...)
	end
	function META:GetContainsAnchor(...)
		return CLIB.btSoftBody_Cluster_getContainsAnchor(self.ptr, ...)
	end
	function META:GetCom(...)
		return CLIB.btSoftBody_Cluster_getCom(self.ptr, ...)
	end
	function META:GetLeaf(...)
		return CLIB.btSoftBody_Cluster_getLeaf(self.ptr, ...)
	end
	function META:GetAv(...)
		return CLIB.btSoftBody_Cluster_getAv(self.ptr, ...)
	end
	function META:GetFramerefs(...)
		return CLIB.btSoftBody_Cluster_getFramerefs(self.ptr, ...)
	end
	function META:GetSelfCollisionImpulseFactor(...)
		return CLIB.btSoftBody_Cluster_getSelfCollisionImpulseFactor(self.ptr, ...)
	end
end
do -- btSliderConstraint
	local META = {}
	library.metatables.btSliderConstraint = META
	META.__index = function(s, k) return META[k] end
	function library.CreateSliderConstraint(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btSliderConstraint_new(...)
		return self
	end
	function library.CreateSliderConstraint2(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btSliderConstraint_new2(...)
		return self
	end
	function META:GetSoftnessDirLin(...)
		return CLIB.btSliderConstraint_getSoftnessDirLin(self.ptr, ...)
	end
	function META:GetFlags(...)
		return CLIB.btSliderConstraint_getFlags(self.ptr, ...)
	end
	function META:GetSoftnessOrthoAng(...)
		return CLIB.btSliderConstraint_getSoftnessOrthoAng(self.ptr, ...)
	end
	function META:GetDampingLimLin(...)
		return CLIB.btSliderConstraint_getDampingLimLin(self.ptr, ...)
	end
	function META:GetUseLinearReferenceFrameA(...)
		return CLIB.btSliderConstraint_getUseLinearReferenceFrameA(self.ptr, ...)
	end
	function META:GetSoftnessOrthoLin(...)
		return CLIB.btSliderConstraint_getSoftnessOrthoLin(self.ptr, ...)
	end
	function META:GetAngularPos(...)
		return CLIB.btSliderConstraint_getAngularPos(self.ptr, ...)
	end
	function META:SetUseFrameOffset(...)
		return CLIB.btSliderConstraint_setUseFrameOffset(self.ptr, ...)
	end
	function META:GetRestitutionOrthoLin(...)
		return CLIB.btSliderConstraint_getRestitutionOrthoLin(self.ptr, ...)
	end
	function META:GetCalculatedTransformA(...)
		return CLIB.btSliderConstraint_getCalculatedTransformA(self.ptr, ...)
	end
	function META:GetDampingOrthoLin(...)
		return CLIB.btSliderConstraint_getDampingOrthoLin(self.ptr, ...)
	end
	function META:GetFrameOffsetA(...)
		return CLIB.btSliderConstraint_getFrameOffsetA(self.ptr, ...)
	end
	function META:GetCalculatedTransformB(...)
		return CLIB.btSliderConstraint_getCalculatedTransformB(self.ptr, ...)
	end
	function META:GetSoftnessDirAng(...)
		return CLIB.btSliderConstraint_getSoftnessDirAng(self.ptr, ...)
	end
	function META:GetMaxAngMotorForce(...)
		return CLIB.btSliderConstraint_getMaxAngMotorForce(self.ptr, ...)
	end
	function META:GetSoftnessLimAng(...)
		return CLIB.btSliderConstraint_getSoftnessLimAng(self.ptr, ...)
	end
	function META:SetMaxAngMotorForce(...)
		return CLIB.btSliderConstraint_setMaxAngMotorForce(self.ptr, ...)
	end
	function META:SetDampingDirLin(...)
		return CLIB.btSliderConstraint_setDampingDirLin(self.ptr, ...)
	end
	function META:SetUpperLinLimit(...)
		return CLIB.btSliderConstraint_setUpperLinLimit(self.ptr, ...)
	end
	function META:GetAncorInB(...)
		return CLIB.btSliderConstraint_getAncorInB(self.ptr, ...)
	end
	function META:GetRestitutionDirLin(...)
		return CLIB.btSliderConstraint_getRestitutionDirLin(self.ptr, ...)
	end
	function META:TestAngLimits(...)
		return CLIB.btSliderConstraint_testAngLimits(self.ptr, ...)
	end
	function META:SetRestitutionDirLin(...)
		return CLIB.btSliderConstraint_setRestitutionDirLin(self.ptr, ...)
	end
	function META:GetDampingLimAng(...)
		return CLIB.btSliderConstraint_getDampingLimAng(self.ptr, ...)
	end
	function META:SetSoftnessLimLin(...)
		return CLIB.btSliderConstraint_setSoftnessLimLin(self.ptr, ...)
	end
	function META:SetLowerLinLimit(...)
		return CLIB.btSliderConstraint_setLowerLinLimit(self.ptr, ...)
	end
	function META:SetSoftnessDirLin(...)
		return CLIB.btSliderConstraint_setSoftnessDirLin(self.ptr, ...)
	end
	function META:SetRestitutionDirAng(...)
		return CLIB.btSliderConstraint_setRestitutionDirAng(self.ptr, ...)
	end
	function META:GetPoweredLinMotor(...)
		return CLIB.btSliderConstraint_getPoweredLinMotor(self.ptr, ...)
	end
	function META:TestLinLimits(...)
		return CLIB.btSliderConstraint_testLinLimits(self.ptr, ...)
	end
	function META:SetUpperAngLimit(...)
		return CLIB.btSliderConstraint_setUpperAngLimit(self.ptr, ...)
	end
	function META:SetTargetLinMotorVelocity(...)
		return CLIB.btSliderConstraint_setTargetLinMotorVelocity(self.ptr, ...)
	end
	function META:SetTargetAngMotorVelocity(...)
		return CLIB.btSliderConstraint_setTargetAngMotorVelocity(self.ptr, ...)
	end
	function META:GetDampingDirLin(...)
		return CLIB.btSliderConstraint_getDampingDirLin(self.ptr, ...)
	end
	function META:SetSoftnessLimAng(...)
		return CLIB.btSliderConstraint_setSoftnessLimAng(self.ptr, ...)
	end
	function META:SetSoftnessDirAng(...)
		return CLIB.btSliderConstraint_setSoftnessDirAng(self.ptr, ...)
	end
	function META:SetRestitutionOrthoLin(...)
		return CLIB.btSliderConstraint_setRestitutionOrthoLin(self.ptr, ...)
	end
	function META:SetPoweredLinMotor(...)
		return CLIB.btSliderConstraint_setPoweredLinMotor(self.ptr, ...)
	end
	function META:SetPoweredAngMotor(...)
		return CLIB.btSliderConstraint_setPoweredAngMotor(self.ptr, ...)
	end
	function META:SetMaxLinMotorForce(...)
		return CLIB.btSliderConstraint_setMaxLinMotorForce(self.ptr, ...)
	end
	function META:SetLowerAngLimit(...)
		return CLIB.btSliderConstraint_setLowerAngLimit(self.ptr, ...)
	end
	function META:CalculateTransforms(...)
		return CLIB.btSliderConstraint_calculateTransforms(self.ptr, ...)
	end
	function META:SetDampingOrthoAng(...)
		return CLIB.btSliderConstraint_setDampingOrthoAng(self.ptr, ...)
	end
	function META:SetDampingLimLin(...)
		return CLIB.btSliderConstraint_setDampingLimLin(self.ptr, ...)
	end
	function META:SetDampingLimAng(...)
		return CLIB.btSliderConstraint_setDampingLimAng(self.ptr, ...)
	end
	function META:SetDampingDirAng(...)
		return CLIB.btSliderConstraint_setDampingDirAng(self.ptr, ...)
	end
	function META:GetUseFrameOffset(...)
		return CLIB.btSliderConstraint_getUseFrameOffset(self.ptr, ...)
	end
	function META:GetUpperLinLimit(...)
		return CLIB.btSliderConstraint_getUpperLinLimit(self.ptr, ...)
	end
	function META:GetUpperAngLimit(...)
		return CLIB.btSliderConstraint_getUpperAngLimit(self.ptr, ...)
	end
	function META:GetTargetLinMotorVelocity(...)
		return CLIB.btSliderConstraint_getTargetLinMotorVelocity(self.ptr, ...)
	end
	function META:GetSolveLinLimit(...)
		return CLIB.btSliderConstraint_getSolveLinLimit(self.ptr, ...)
	end
	function META:GetSolveAngLimit(...)
		return CLIB.btSliderConstraint_getSolveAngLimit(self.ptr, ...)
	end
	function META:GetSoftnessLimLin(...)
		return CLIB.btSliderConstraint_getSoftnessLimLin(self.ptr, ...)
	end
	function META:GetTargetAngMotorVelocity(...)
		return CLIB.btSliderConstraint_getTargetAngMotorVelocity(self.ptr, ...)
	end
	function META:GetRestitutionOrthoAng(...)
		return CLIB.btSliderConstraint_getRestitutionOrthoAng(self.ptr, ...)
	end
	function META:GetRestitutionDirAng(...)
		return CLIB.btSliderConstraint_getRestitutionDirAng(self.ptr, ...)
	end
	function META:GetPoweredAngMotor(...)
		return CLIB.btSliderConstraint_getPoweredAngMotor(self.ptr, ...)
	end
	function META:GetMaxLinMotorForce(...)
		return CLIB.btSliderConstraint_getMaxLinMotorForce(self.ptr, ...)
	end
	function META:GetLowerLinLimit(...)
		return CLIB.btSliderConstraint_getLowerLinLimit(self.ptr, ...)
	end
	function META:GetLowerAngLimit(...)
		return CLIB.btSliderConstraint_getLowerAngLimit(self.ptr, ...)
	end
	function META:GetLinDepth(...)
		return CLIB.btSliderConstraint_getLinDepth(self.ptr, ...)
	end
	function META:GetInfo2NonVirtual(...)
		return CLIB.btSliderConstraint_getInfo2NonVirtual(self.ptr, ...)
	end
	function META:GetInfo1NonVirtual(...)
		return CLIB.btSliderConstraint_getInfo1NonVirtual(self.ptr, ...)
	end
	function META:GetDampingOrthoAng(...)
		return CLIB.btSliderConstraint_getDampingOrthoAng(self.ptr, ...)
	end
	function META:SetSoftnessOrthoAng(...)
		return CLIB.btSliderConstraint_setSoftnessOrthoAng(self.ptr, ...)
	end
	function META:GetDampingDirAng(...)
		return CLIB.btSliderConstraint_getDampingDirAng(self.ptr, ...)
	end
	function META:GetAngDepth(...)
		return CLIB.btSliderConstraint_getAngDepth(self.ptr, ...)
	end
	function META:GetAncorInA(...)
		return CLIB.btSliderConstraint_getAncorInA(self.ptr, ...)
	end
	function META:SetDampingOrthoLin(...)
		return CLIB.btSliderConstraint_setDampingOrthoLin(self.ptr, ...)
	end
	function META:GetLinearPos(...)
		return CLIB.btSliderConstraint_getLinearPos(self.ptr, ...)
	end
	function META:SetRestitutionOrthoAng(...)
		return CLIB.btSliderConstraint_setRestitutionOrthoAng(self.ptr, ...)
	end
	function META:GetRestitutionLimAng(...)
		return CLIB.btSliderConstraint_getRestitutionLimAng(self.ptr, ...)
	end
	function META:SetSoftnessOrthoLin(...)
		return CLIB.btSliderConstraint_setSoftnessOrthoLin(self.ptr, ...)
	end
	function META:SetRestitutionLimAng(...)
		return CLIB.btSliderConstraint_setRestitutionLimAng(self.ptr, ...)
	end
	function META:GetRestitutionLimLin(...)
		return CLIB.btSliderConstraint_getRestitutionLimLin(self.ptr, ...)
	end
	function META:SetFrames(...)
		return CLIB.btSliderConstraint_setFrames(self.ptr, ...)
	end
	function META:SetRestitutionLimLin(...)
		return CLIB.btSliderConstraint_setRestitutionLimLin(self.ptr, ...)
	end
	function META:GetFrameOffsetB(...)
		return CLIB.btSliderConstraint_getFrameOffsetB(self.ptr, ...)
	end
end
do -- btSoftBody_Impulse
	local META = {}
	library.metatables.btSoftBody_Impulse = META
	META.__index = function(s, k) return META[k] end
	function library.CreateSoftBody_Impulse(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btSoftBody_Impulse_new(...)
		return self
	end
	function META:Delete(...)
		return CLIB.btSoftBody_Impulse_delete(self.ptr, ...)
	end
	function META:GetAsDrift(...)
		return CLIB.btSoftBody_Impulse_getAsDrift(self.ptr, ...)
	end
	function META:SetAsDrift(...)
		return CLIB.btSoftBody_Impulse_setAsDrift(self.ptr, ...)
	end
	function META:GetDrift(...)
		return CLIB.btSoftBody_Impulse_getDrift(self.ptr, ...)
	end
	function META:SetVelocity(...)
		return CLIB.btSoftBody_Impulse_setVelocity(self.ptr, ...)
	end
	function META:SetDrift(...)
		return CLIB.btSoftBody_Impulse_setDrift(self.ptr, ...)
	end
	function META:SetAsVelocity(...)
		return CLIB.btSoftBody_Impulse_setAsVelocity(self.ptr, ...)
	end
	function META:GetAsVelocity(...)
		return CLIB.btSoftBody_Impulse_getAsVelocity(self.ptr, ...)
	end
	function META:GetVelocity(...)
		return CLIB.btSoftBody_Impulse_getVelocity(self.ptr, ...)
	end
end
do -- btOptimizedBvhNode
	local META = {}
	library.metatables.btOptimizedBvhNode = META
	META.__index = function(s, k) return META[k] end
	function library.CreateOptimizedBvhNode(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btOptimizedBvhNode_new(...)
		return self
	end
	function META:GetSubPart(...)
		return CLIB.btOptimizedBvhNode_getSubPart(self.ptr, ...)
	end
	function META:GetTriangleIndex(...)
		return CLIB.btOptimizedBvhNode_getTriangleIndex(self.ptr, ...)
	end
	function META:Delete(...)
		return CLIB.btOptimizedBvhNode_delete(self.ptr, ...)
	end
	function META:SetTriangleIndex(...)
		return CLIB.btOptimizedBvhNode_setTriangleIndex(self.ptr, ...)
	end
	function META:SetEscapeIndex(...)
		return CLIB.btOptimizedBvhNode_setEscapeIndex(self.ptr, ...)
	end
	function META:SetSubPart(...)
		return CLIB.btOptimizedBvhNode_setSubPart(self.ptr, ...)
	end
	function META:SetAabbMinOrg(...)
		return CLIB.btOptimizedBvhNode_setAabbMinOrg(self.ptr, ...)
	end
	function META:SetAabbMaxOrg(...)
		return CLIB.btOptimizedBvhNode_setAabbMaxOrg(self.ptr, ...)
	end
	function META:GetEscapeIndex(...)
		return CLIB.btOptimizedBvhNode_getEscapeIndex(self.ptr, ...)
	end
	function META:GetAabbMinOrg(...)
		return CLIB.btOptimizedBvhNode_getAabbMinOrg(self.ptr, ...)
	end
	function META:GetAabbMaxOrg(...)
		return CLIB.btOptimizedBvhNode_getAabbMaxOrg(self.ptr, ...)
	end
end
do -- btQuantizedBvhTree
	local META = {}
	library.metatables.btQuantizedBvhTree = META
	META.__index = function(s, k) return META[k] end
	function library.CreateQuantizedBvhTree(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btQuantizedBvhTree_new(...)
		return self
	end
	function META:TestQuantizedBoxOverlapp(...)
		return CLIB.btQuantizedBvhTree_testQuantizedBoxOverlapp(self.ptr, ...)
	end
	function META:Delete(...)
		return CLIB.btQuantizedBvhTree_delete(self.ptr, ...)
	end
	function META:IsLeafNode(...)
		return CLIB.btQuantizedBvhTree_isLeafNode(self.ptr, ...)
	end
	function META:GetRightNode(...)
		return CLIB.btQuantizedBvhTree_getRightNode(self.ptr, ...)
	end
	function META:GetNodeCount(...)
		return CLIB.btQuantizedBvhTree_getNodeCount(self.ptr, ...)
	end
	function META:GetLeftNode(...)
		return CLIB.btQuantizedBvhTree_getLeftNode(self.ptr, ...)
	end
	function META:ClearNodes(...)
		return CLIB.btQuantizedBvhTree_clearNodes(self.ptr, ...)
	end
	function META:GetNodeBound(...)
		return CLIB.btQuantizedBvhTree_getNodeBound(self.ptr, ...)
	end
	function META:GetEscapeNodeIndex(...)
		return CLIB.btQuantizedBvhTree_getEscapeNodeIndex(self.ptr, ...)
	end
	function META:GetNodeData(...)
		return CLIB.btQuantizedBvhTree_getNodeData(self.ptr, ...)
	end
	function META:SetNodeBound(...)
		return CLIB.btQuantizedBvhTree_setNodeBound(self.ptr, ...)
	end
	function META:QuantizePoint(...)
		return CLIB.btQuantizedBvhTree_quantizePoint(self.ptr, ...)
	end
end
do -- btGImpactMeshShape
	local META = {}
	library.metatables.btGImpactMeshShape = META
	META.__index = function(s, k) return META[k] end
	function library.CreateGImpactMeshShape(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btGImpactMeshShape_new(...)
		return self
	end
	function META:GetMeshInterface(...)
		return CLIB.btGImpactMeshShape_getMeshInterface(self.ptr, ...)
	end
	function META:GetMeshPartCount(...)
		return CLIB.btGImpactMeshShape_getMeshPartCount(self.ptr, ...)
	end
	function META:GetMeshPart(...)
		return CLIB.btGImpactMeshShape_getMeshPart(self.ptr, ...)
	end
end
do -- btHinge2Constraint
	local META = {}
	library.metatables.btHinge2Constraint = META
	META.__index = function(s, k) return META[k] end
	function library.CreateHinge2Constraint(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btHinge2Constraint_new(...)
		return self
	end
	function META:GetAnchor(...)
		return CLIB.btHinge2Constraint_getAnchor(self.ptr, ...)
	end
	function META:GetAxis2(...)
		return CLIB.btHinge2Constraint_getAxis2(self.ptr, ...)
	end
	function META:GetAxis1(...)
		return CLIB.btHinge2Constraint_getAxis1(self.ptr, ...)
	end
	function META:GetAngle2(...)
		return CLIB.btHinge2Constraint_getAngle2(self.ptr, ...)
	end
	function META:SetLowerLimit(...)
		return CLIB.btHinge2Constraint_setLowerLimit(self.ptr, ...)
	end
	function META:GetAngle1(...)
		return CLIB.btHinge2Constraint_getAngle1(self.ptr, ...)
	end
	function META:SetUpperLimit(...)
		return CLIB.btHinge2Constraint_setUpperLimit(self.ptr, ...)
	end
	function META:GetAnchor2(...)
		return CLIB.btHinge2Constraint_getAnchor2(self.ptr, ...)
	end
end
do -- btConstraintSolver
	local META = {}
	library.metatables.btConstraintSolver = META
	META.__index = function(s, k) return META[k] end
	function META:Reset(...)
		return CLIB.btConstraintSolver_reset(self.ptr, ...)
	end
	function META:AllSolved(...)
		return CLIB.btConstraintSolver_allSolved(self.ptr, ...)
	end
	function META:SolveGroup(...)
		return CLIB.btConstraintSolver_solveGroup(self.ptr, ...)
	end
	function META:PrepareSolve(...)
		return CLIB.btConstraintSolver_prepareSolve(self.ptr, ...)
	end
	function META:Delete(...)
		return CLIB.btConstraintSolver_delete(self.ptr, ...)
	end
	function META:GetSolverType(...)
		return CLIB.btConstraintSolver_getSolverType(self.ptr, ...)
	end
end
do -- btMultiSphereShape
	local META = {}
	library.metatables.btMultiSphereShape = META
	META.__index = function(s, k) return META[k] end
	function library.CreateMultiSphereShape(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btMultiSphereShape_new2(...)
		return self
	end
	function library.CreateMultiSphereShape2(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btMultiSphereShape_new(...)
		return self
	end
	function META:GetSphereRadius(...)
		return CLIB.btMultiSphereShape_getSphereRadius(self.ptr, ...)
	end
	function META:GetSphereCount(...)
		return CLIB.btMultiSphereShape_getSphereCount(self.ptr, ...)
	end
	function META:GetSpherePosition(...)
		return CLIB.btMultiSphereShape_getSpherePosition(self.ptr, ...)
	end
end
do -- btVehicleRaycaster
	local META = {}
	library.metatables.btVehicleRaycaster = META
	META.__index = function(s, k) return META[k] end
	function META:Delete(...)
		return CLIB.btVehicleRaycaster_delete(self.ptr, ...)
	end
	function META:CastRay(...)
		return CLIB.btVehicleRaycaster_castRay(self.ptr, ...)
	end
end
do -- btConvexPolyhedron
	local META = {}
	library.metatables.btConvexPolyhedron = META
	META.__index = function(s, k) return META[k] end
	function library.CreateConvexPolyhedron(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btConvexPolyhedron_new(...)
		return self
	end
	function META:Initialize(...)
		return CLIB.btConvexPolyhedron_initialize(self.ptr, ...)
	end
	function META:GetVertices(...)
		return CLIB.btConvexPolyhedron_getVertices(self.ptr, ...)
	end
	function META:SetLocalCenter(...)
		return CLIB.btConvexPolyhedron_setLocalCenter(self.ptr, ...)
	end
	function META:SetMC(...)
		return CLIB.btConvexPolyhedron_setMC(self.ptr, ...)
	end
	function META:SetExtents(...)
		return CLIB.btConvexPolyhedron_setExtents(self.ptr, ...)
	end
	function META:TestContainment(...)
		return CLIB.btConvexPolyhedron_testContainment(self.ptr, ...)
	end
	function META:GetME(...)
		return CLIB.btConvexPolyhedron_getME(self.ptr, ...)
	end
	function META:GetExtents(...)
		return CLIB.btConvexPolyhedron_getExtents(self.ptr, ...)
	end
	function META:GetFaces(...)
		return CLIB.btConvexPolyhedron_getFaces(self.ptr, ...)
	end
	function META:GetLocalCenter(...)
		return CLIB.btConvexPolyhedron_getLocalCenter(self.ptr, ...)
	end
	function META:SetME(...)
		return CLIB.btConvexPolyhedron_setME(self.ptr, ...)
	end
	function META:GetRadius(...)
		return CLIB.btConvexPolyhedron_getRadius(self.ptr, ...)
	end
	function META:GetMC(...)
		return CLIB.btConvexPolyhedron_getMC(self.ptr, ...)
	end
	function META:SetRadius(...)
		return CLIB.btConvexPolyhedron_setRadius(self.ptr, ...)
	end
	function META:Delete(...)
		return CLIB.btConvexPolyhedron_delete(self.ptr, ...)
	end
	function META:Project(...)
		return CLIB.btConvexPolyhedron_project(self.ptr, ...)
	end
	function META:GetUniqueEdges(...)
		return CLIB.btConvexPolyhedron_getUniqueEdges(self.ptr, ...)
	end
end
do -- btBvhTree_get_node
	local META = {}
	library.metatables.btBvhTree_get_node = META
	META.__index = function(s, k) return META[k] end
	function META:Pointer(...)
		return CLIB.btBvhTree_get_node_pointer(self.ptr, ...)
	end
	function META:Pointer2(...)
		return CLIB.btBvhTree_get_node_pointer2(self.ptr, ...)
	end
end
do -- btSoftBody_Feature
	local META = {}
	library.metatables.btSoftBody_Feature = META
	META.__index = function(s, k) return META[k] end
	function META:GetMaterial(...)
		return CLIB.btSoftBody_Feature_getMaterial(self.ptr, ...)
	end
	function META:SetMaterial(...)
		return CLIB.btSoftBody_Feature_setMaterial(self.ptr, ...)
	end
end
do -- btDbvt_array_index
	local META = {}
	library.metatables.btDbvt_array_index = META
	META.__index = function(s, k) return META[k] end
	function META:Of(...)
		return CLIB.btDbvt_array_index_of(self.ptr, ...)
	end
end
do -- btStaticPlaneShape
	local META = {}
	library.metatables.btStaticPlaneShape = META
	META.__index = function(s, k) return META[k] end
	function library.CreateStaticPlaneShape(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btStaticPlaneShape_new(...)
		return self
	end
	function META:GetPlaneNormal(...)
		return CLIB.btStaticPlaneShape_getPlaneNormal(self.ptr, ...)
	end
	function META:GetPlaneConstant(...)
		return CLIB.btStaticPlaneShape_getPlaneConstant(self.ptr, ...)
	end
end
do -- btTriangleCallback
	local META = {}
	library.metatables.btTriangleCallback = META
	META.__index = function(s, k) return META[k] end
	function META:Delete(...)
		return CLIB.btTriangleCallback_delete(self.ptr, ...)
	end
end
do -- btHingeConstraint
	local META = {}
	library.metatables.btHingeConstraint = META
	META.__index = function(s, k) return META[k] end
	function library.CreateHingeConstraint(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btHingeConstraint_new(...)
		return self
	end
	function library.CreateHingeConstraint2(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btHingeConstraint_new3(...)
		return self
	end
	function library.CreateHingeConstraint3(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btHingeConstraint_new2(...)
		return self
	end
	function library.CreateHingeConstraint4(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btHingeConstraint_new4(...)
		return self
	end
	function META:GetInfo2InternalUsingFrameOffset(...)
		return CLIB.btHingeConstraint_getInfo2InternalUsingFrameOffset(self.ptr, ...)
	end
	function META:SetLimit2(...)
		return CLIB.btHingeConstraint_setLimit2(self.ptr, ...)
	end
	function META:GetLimitBiasFactor(...)
		return CLIB.btHingeConstraint_getLimitBiasFactor(self.ptr, ...)
	end
	function META:GetLowerLimit(...)
		return CLIB.btHingeConstraint_getLowerLimit(self.ptr, ...)
	end
	function META:SetMotorTarget(...)
		return CLIB.btHingeConstraint_setMotorTarget(self.ptr, ...)
	end
	function META:GetFrameOffsetB(...)
		return CLIB.btHingeConstraint_getFrameOffsetB(self.ptr, ...)
	end
	function META:SetLimit3(...)
		return CLIB.btHingeConstraint_setLimit3(self.ptr, ...)
	end
	function META:SetMotorTarget2(...)
		return CLIB.btHingeConstraint_setMotorTarget2(self.ptr, ...)
	end
	function META:GetHingeAngle(...)
		return CLIB.btHingeConstraint_getHingeAngle(self.ptr, ...)
	end
	function META:SetAxis(...)
		return CLIB.btHingeConstraint_setAxis(self.ptr, ...)
	end
	function META:HasLimit(...)
		return CLIB.btHingeConstraint_hasLimit(self.ptr, ...)
	end
	function META:SetLimit4(...)
		return CLIB.btHingeConstraint_setLimit4(self.ptr, ...)
	end
	function META:GetAFrame(...)
		return CLIB.btHingeConstraint_getAFrame(self.ptr, ...)
	end
	function META:GetMotorTargetVelocity(...)
		return CLIB.btHingeConstraint_getMotorTargetVelocity(self.ptr, ...)
	end
	function META:GetUseReferenceFrameA(...)
		return CLIB.btHingeConstraint_getUseReferenceFrameA(self.ptr, ...)
	end
	function META:UpdateRHS(...)
		return CLIB.btHingeConstraint_updateRHS(self.ptr, ...)
	end
	function META:SetUseFrameOffset(...)
		return CLIB.btHingeConstraint_setUseFrameOffset(self.ptr, ...)
	end
	function META:GetFlags(...)
		return CLIB.btHingeConstraint_getFlags(self.ptr, ...)
	end
	function META:SetMaxMotorImpulse(...)
		return CLIB.btHingeConstraint_setMaxMotorImpulse(self.ptr, ...)
	end
	function META:SetUseReferenceFrameA(...)
		return CLIB.btHingeConstraint_setUseReferenceFrameA(self.ptr, ...)
	end
	function META:SetLimit(...)
		return CLIB.btHingeConstraint_setLimit(self.ptr, ...)
	end
	function META:GetInfo2NonVirtual(...)
		return CLIB.btHingeConstraint_getInfo2NonVirtual(self.ptr, ...)
	end
	function META:GetLimitRelaxationFactor(...)
		return CLIB.btHingeConstraint_getLimitRelaxationFactor(self.ptr, ...)
	end
	function META:EnableAngularMotor(...)
		return CLIB.btHingeConstraint_enableAngularMotor(self.ptr, ...)
	end
	function META:GetAngularOnly(...)
		return CLIB.btHingeConstraint_getAngularOnly(self.ptr, ...)
	end
	function META:GetLimitSoftness(...)
		return CLIB.btHingeConstraint_getLimitSoftness(self.ptr, ...)
	end
	function META:SetFrames(...)
		return CLIB.btHingeConstraint_setFrames(self.ptr, ...)
	end
	function META:GetUseFrameOffset(...)
		return CLIB.btHingeConstraint_getUseFrameOffset(self.ptr, ...)
	end
	function META:GetEnableAngularMotor(...)
		return CLIB.btHingeConstraint_getEnableAngularMotor(self.ptr, ...)
	end
	function META:SetAngularOnly(...)
		return CLIB.btHingeConstraint_setAngularOnly(self.ptr, ...)
	end
	function META:GetHingeAngle2(...)
		return CLIB.btHingeConstraint_getHingeAngle2(self.ptr, ...)
	end
	function META:GetBFrame(...)
		return CLIB.btHingeConstraint_getBFrame(self.ptr, ...)
	end
	function META:EnableMotor(...)
		return CLIB.btHingeConstraint_enableMotor(self.ptr, ...)
	end
	function META:SetMotorTargetVelocity(...)
		return CLIB.btHingeConstraint_setMotorTargetVelocity(self.ptr, ...)
	end
	function META:GetLimitSign(...)
		return CLIB.btHingeConstraint_getLimitSign(self.ptr, ...)
	end
	function META:GetMaxMotorImpulse(...)
		return CLIB.btHingeConstraint_getMaxMotorImpulse(self.ptr, ...)
	end
	function META:GetFrameOffsetA(...)
		return CLIB.btHingeConstraint_getFrameOffsetA(self.ptr, ...)
	end
	function META:GetInfo1NonVirtual(...)
		return CLIB.btHingeConstraint_getInfo1NonVirtual(self.ptr, ...)
	end
	function META:GetInfo2Internal(...)
		return CLIB.btHingeConstraint_getInfo2Internal(self.ptr, ...)
	end
	function META:GetSolveLimit(...)
		return CLIB.btHingeConstraint_getSolveLimit(self.ptr, ...)
	end
	function META:GetUpperLimit(...)
		return CLIB.btHingeConstraint_getUpperLimit(self.ptr, ...)
	end
	function META:TestLimit(...)
		return CLIB.btHingeConstraint_testLimit(self.ptr, ...)
	end
end
do -- btTypedConstraint
	local META = {}
	library.metatables.btTypedConstraint = META
	META.__index = function(s, k) return META[k] end
	function META:GetUid(...)
		return CLIB.btTypedConstraint_getUid(self.ptr, ...)
	end
	function META:Delete(...)
		return CLIB.btTypedConstraint_delete(self.ptr, ...)
	end
	function META:IsEnabled(...)
		return CLIB.btTypedConstraint_isEnabled(self.ptr, ...)
	end
	function META:GetUserConstraintPtr(...)
		return CLIB.btTypedConstraint_getUserConstraintPtr(self.ptr, ...)
	end
	function META:SetParam2(...)
		return CLIB.btTypedConstraint_setParam2(self.ptr, ...)
	end
	function META:GetUserConstraintId(...)
		return CLIB.btTypedConstraint_getUserConstraintId(self.ptr, ...)
	end
	function META:GetUserConstraintType(...)
		return CLIB.btTypedConstraint_getUserConstraintType(self.ptr, ...)
	end
	function META:SetUserConstraintType(...)
		return CLIB.btTypedConstraint_setUserConstraintType(self.ptr, ...)
	end
	function META:SetUserConstraintPtr(...)
		return CLIB.btTypedConstraint_setUserConstraintPtr(self.ptr, ...)
	end
	function META:SetupSolverConstraint(...)
		return CLIB.btTypedConstraint_setupSolverConstraint(self.ptr, ...)
	end
	function META:SetParam(...)
		return CLIB.btTypedConstraint_setParam(self.ptr, ...)
	end
	function META:SetOverrideNumSolverIterations(...)
		return CLIB.btTypedConstraint_setOverrideNumSolverIterations(self.ptr, ...)
	end
	function META:SetJointFeedback(...)
		return CLIB.btTypedConstraint_setJointFeedback(self.ptr, ...)
	end
	function META:SetEnabled(...)
		return CLIB.btTypedConstraint_setEnabled(self.ptr, ...)
	end
	function META:SetDbgDrawSize(...)
		return CLIB.btTypedConstraint_setDbgDrawSize(self.ptr, ...)
	end
	function META:SetBreakingImpulseThreshold(...)
		return CLIB.btTypedConstraint_setBreakingImpulseThreshold(self.ptr, ...)
	end
	function META:Serialize(...)
		return CLIB.btTypedConstraint_serialize(self.ptr, ...)
	end
	function META:InternalSetAppliedImpulse(...)
		return CLIB.btTypedConstraint_internalSetAppliedImpulse(self.ptr, ...)
	end
	function META:GetRigidBodyB(...)
		return CLIB.btTypedConstraint_getRigidBodyB(self.ptr, ...)
	end
	function META:GetParam2(...)
		return CLIB.btTypedConstraint_getParam2(self.ptr, ...)
	end
	function META:GetParam(...)
		return CLIB.btTypedConstraint_getParam(self.ptr, ...)
	end
	function META:GetJointFeedback(...)
		return CLIB.btTypedConstraint_getJointFeedback(self.ptr, ...)
	end
	function META:GetInfo2(...)
		return CLIB.btTypedConstraint_getInfo2(self.ptr, ...)
	end
	function META:GetInfo1(...)
		return CLIB.btTypedConstraint_getInfo1(self.ptr, ...)
	end
	function META:GetConstraintType(...)
		return CLIB.btTypedConstraint_getConstraintType(self.ptr, ...)
	end
	function META:GetBreakingImpulseThreshold(...)
		return CLIB.btTypedConstraint_getBreakingImpulseThreshold(self.ptr, ...)
	end
	function META:GetAppliedImpulse(...)
		return CLIB.btTypedConstraint_getAppliedImpulse(self.ptr, ...)
	end
	function META:CalculateSerializeBufferSize(...)
		return CLIB.btTypedConstraint_calculateSerializeBufferSize(self.ptr, ...)
	end
	function META:BuildJacobian(...)
		return CLIB.btTypedConstraint_buildJacobian(self.ptr, ...)
	end
	function META:NeedsFeedback(...)
		return CLIB.btTypedConstraint_needsFeedback(self.ptr, ...)
	end
	function META:GetDbgDrawSize(...)
		return CLIB.btTypedConstraint_getDbgDrawSize(self.ptr, ...)
	end
	function META:GetOverrideNumSolverIterations(...)
		return CLIB.btTypedConstraint_getOverrideNumSolverIterations(self.ptr, ...)
	end
	function META:SetUserConstraintId(...)
		return CLIB.btTypedConstraint_setUserConstraintId(self.ptr, ...)
	end
	function META:SolveConstraintObsolete(...)
		return CLIB.btTypedConstraint_solveConstraintObsolete(self.ptr, ...)
	end
	function META:GetFixedBody(...)
		return CLIB.btTypedConstraint_getFixedBody(self.ptr, ...)
	end
	function META:EnableFeedback(...)
		return CLIB.btTypedConstraint_enableFeedback(self.ptr, ...)
	end
	function META:InternalGetAppliedImpulse(...)
		return CLIB.btTypedConstraint_internalGetAppliedImpulse(self.ptr, ...)
	end
	function META:GetRigidBodyA(...)
		return CLIB.btTypedConstraint_getRigidBodyA(self.ptr, ...)
	end
end
do -- bt32BitAxisSweep3
	local META = {}
	library.metatables.bt32BitAxisSweep3 = META
	META.__index = function(s, k) return META[k] end
	function library.Create32BitAxisSweep3(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.bt32BitAxisSweep3_new(...)
		return self
	end
	function META:GetOverlappingPairUserCallback(...)
		return CLIB.bt32BitAxisSweep3_getOverlappingPairUserCallback(self.ptr, ...)
	end
	function META:GetNumHandles(...)
		return CLIB.bt32BitAxisSweep3_getNumHandles(self.ptr, ...)
	end
	function META:SetOverlappingPairUserCallback(...)
		return CLIB.bt32BitAxisSweep3_setOverlappingPairUserCallback(self.ptr, ...)
	end
	function META:UnQuantize(...)
		return CLIB.bt32BitAxisSweep3_unQuantize(self.ptr, ...)
	end
	function META:TestAabbOverlap(...)
		return CLIB.bt32BitAxisSweep3_testAabbOverlap(self.ptr, ...)
	end
	function META:AddHandle(...)
		return CLIB.bt32BitAxisSweep3_addHandle(self.ptr, ...)
	end
	function META:UpdateHandle(...)
		return CLIB.bt32BitAxisSweep3_updateHandle(self.ptr, ...)
	end
	function META:Quantize(...)
		return CLIB.bt32BitAxisSweep3_quantize(self.ptr, ...)
	end
	function META:GetHandle(...)
		return CLIB.bt32BitAxisSweep3_getHandle(self.ptr, ...)
	end
	function META:RemoveHandle(...)
		return CLIB.bt32BitAxisSweep3_removeHandle(self.ptr, ...)
	end
end
do -- btOverlapCallback
	local META = {}
	library.metatables.btOverlapCallback = META
	META.__index = function(s, k) return META[k] end
	function META:ProcessOverlap(...)
		return CLIB.btOverlapCallback_processOverlap(self.ptr, ...)
	end
	function META:Delete(...)
		return CLIB.btOverlapCallback_delete(self.ptr, ...)
	end
end
do -- btConvexHullShape
	local META = {}
	library.metatables.btConvexHullShape = META
	META.__index = function(s, k) return META[k] end
	function library.CreateConvexHullShape(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btConvexHullShape_new2(...)
		return self
	end
	function library.CreateConvexHullShape2(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btConvexHullShape_new3(...)
		return self
	end
	function library.CreateConvexHullShape3(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btConvexHullShape_new4(...)
		return self
	end
	function library.CreateConvexHullShape4(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btConvexHullShape_new(...)
		return self
	end
	function META:GetScaledPoint(...)
		return CLIB.btConvexHullShape_getScaledPoint(self.ptr, ...)
	end
	function META:OptimizeConvexHull(...)
		return CLIB.btConvexHullShape_optimizeConvexHull(self.ptr, ...)
	end
	function META:AddPoint(...)
		return CLIB.btConvexHullShape_addPoint(self.ptr, ...)
	end
	function META:GetUnscaledPoints(...)
		return CLIB.btConvexHullShape_getUnscaledPoints(self.ptr, ...)
	end
	function META:GetNumPoints(...)
		return CLIB.btConvexHullShape_getNumPoints(self.ptr, ...)
	end
end
do -- btGjkPairDetector
	local META = {}
	library.metatables.btGjkPairDetector = META
	META.__index = function(s, k) return META[k] end
	function library.CreateGjkPairDetector(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btGjkPairDetector_new2(...)
		return self
	end
	function library.CreateGjkPairDetector2(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btGjkPairDetector_new(...)
		return self
	end
	function META:SetDegenerateSimplex(...)
		return CLIB.btGjkPairDetector_setDegenerateSimplex(self.ptr, ...)
	end
	function META:SetCurIter(...)
		return CLIB.btGjkPairDetector_setCurIter(self.ptr, ...)
	end
	function META:GetCurIter(...)
		return CLIB.btGjkPairDetector_getCurIter(self.ptr, ...)
	end
	function META:GetCatchDegeneracies(...)
		return CLIB.btGjkPairDetector_getCatchDegeneracies(self.ptr, ...)
	end
	function META:GetClosestPointsNonVirtual(...)
		return CLIB.btGjkPairDetector_getClosestPointsNonVirtual(self.ptr, ...)
	end
	function META:GetCachedSeparatingAxis(...)
		return CLIB.btGjkPairDetector_getCachedSeparatingAxis(self.ptr, ...)
	end
	function META:GetLastUsedMethod(...)
		return CLIB.btGjkPairDetector_getLastUsedMethod(self.ptr, ...)
	end
	function META:SetPenetrationDepthSolver(...)
		return CLIB.btGjkPairDetector_setPenetrationDepthSolver(self.ptr, ...)
	end
	function META:SetCachedSeparatingAxis(...)
		return CLIB.btGjkPairDetector_setCachedSeparatingAxis(self.ptr, ...)
	end
	function META:SetFixContactNormalDirection(...)
		return CLIB.btGjkPairDetector_setFixContactNormalDirection(self.ptr, ...)
	end
	function META:GetFixContactNormalDirection(...)
		return CLIB.btGjkPairDetector_getFixContactNormalDirection(self.ptr, ...)
	end
	function META:GetCachedSeparatingDistance(...)
		return CLIB.btGjkPairDetector_getCachedSeparatingDistance(self.ptr, ...)
	end
	function META:SetCatchDegeneracies(...)
		return CLIB.btGjkPairDetector_setCatchDegeneracies(self.ptr, ...)
	end
	function META:SetLastUsedMethod(...)
		return CLIB.btGjkPairDetector_setLastUsedMethod(self.ptr, ...)
	end
	function META:SetMinkowskiA(...)
		return CLIB.btGjkPairDetector_setMinkowskiA(self.ptr, ...)
	end
	function META:GetDegenerateSimplex(...)
		return CLIB.btGjkPairDetector_getDegenerateSimplex(self.ptr, ...)
	end
	function META:SetIgnoreMargin(...)
		return CLIB.btGjkPairDetector_setIgnoreMargin(self.ptr, ...)
	end
	function META:SetMinkowskiB(...)
		return CLIB.btGjkPairDetector_setMinkowskiB(self.ptr, ...)
	end
end
do -- btAABB_projection
	local META = {}
	library.metatables.btAABB_projection = META
	META.__index = function(s, k) return META[k] end
	function META:Interval(...)
		return CLIB.btAABB_projection_interval(self.ptr, ...)
	end
end
do -- btSoftBody_VSolve
	local META = {}
	library.metatables.btSoftBody_VSolve = META
	META.__index = function(s, k) return META[k] end
	function META:Links(...)
		return CLIB.btSoftBody_VSolve_Links(self.ptr, ...)
	end
end
do -- btBroadphaseProxy
	local META = {}
	library.metatables.btBroadphaseProxy = META
	META.__index = function(s, k) return META[k] end
	function META:SetClientObject(...)
		return CLIB.btBroadphaseProxy_setClientObject(self.ptr, ...)
	end
	function META:SetAabbMax(...)
		return CLIB.btBroadphaseProxy_setAabbMax(self.ptr, ...)
	end
	function META:SetAabbMin(...)
		return CLIB.btBroadphaseProxy_setAabbMin(self.ptr, ...)
	end
	function META:GetUniqueId(...)
		return CLIB.btBroadphaseProxy_getUniqueId(self.ptr, ...)
	end
	function META:IsNonMoving(...)
		return CLIB.btBroadphaseProxy_isNonMoving(self.ptr, ...)
	end
	function META:GetCollisionFilterGroup(...)
		return CLIB.btBroadphaseProxy_getCollisionFilterGroup(self.ptr, ...)
	end
	function META:IsPolyhedral(...)
		return CLIB.btBroadphaseProxy_isPolyhedral(self.ptr, ...)
	end
	function META:GetClientObject(...)
		return CLIB.btBroadphaseProxy_getClientObject(self.ptr, ...)
	end
	function META:IsCompound(...)
		return CLIB.btBroadphaseProxy_isCompound(self.ptr, ...)
	end
	function META:IsInfinite(...)
		return CLIB.btBroadphaseProxy_isInfinite(self.ptr, ...)
	end
	function META:GetAabbMin(...)
		return CLIB.btBroadphaseProxy_getAabbMin(self.ptr, ...)
	end
	function META:IsConvex(...)
		return CLIB.btBroadphaseProxy_isConvex(self.ptr, ...)
	end
	function META:IsConvex2d(...)
		return CLIB.btBroadphaseProxy_isConvex2d(self.ptr, ...)
	end
	function META:IsSoftBody(...)
		return CLIB.btBroadphaseProxy_isSoftBody(self.ptr, ...)
	end
	function META:SetUniqueId(...)
		return CLIB.btBroadphaseProxy_setUniqueId(self.ptr, ...)
	end
	function META:SetCollisionFilterGroup(...)
		return CLIB.btBroadphaseProxy_setCollisionFilterGroup(self.ptr, ...)
	end
	function META:Delete(...)
		return CLIB.btBroadphaseProxy_delete(self.ptr, ...)
	end
	function META:IsConcave(...)
		return CLIB.btBroadphaseProxy_isConcave(self.ptr, ...)
	end
	function META:GetAabbMax(...)
		return CLIB.btBroadphaseProxy_getAabbMax(self.ptr, ...)
	end
	function META:GetCollisionFilterMask(...)
		return CLIB.btBroadphaseProxy_getCollisionFilterMask(self.ptr, ...)
	end
	function META:SetCollisionFilterMask(...)
		return CLIB.btBroadphaseProxy_setCollisionFilterMask(self.ptr, ...)
	end
	function META:GetUid(...)
		return CLIB.btBroadphaseProxy_getUid(self.ptr, ...)
	end
end
do -- btSoftBody_AJoint
	local META = {}
	library.metatables.btSoftBody_AJoint = META
	META.__index = function(s, k) return META[k] end
	function META:SetIcontrol(...)
		return CLIB.btSoftBody_AJoint_setIcontrol(self.ptr, ...)
	end
	function META:GetIcontrol(...)
		return CLIB.btSoftBody_AJoint_getIcontrol(self.ptr, ...)
	end
	function META:GetAxis(...)
		return CLIB.btSoftBody_AJoint_getAxis(self.ptr, ...)
	end
end
do -- btTriangleShapeEx
	local META = {}
	library.metatables.btTriangleShapeEx = META
	META.__index = function(s, k) return META[k] end
	function library.CreateTriangleShapeEx(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btTriangleShapeEx_new2(...)
		return self
	end
	function library.CreateTriangleShapeEx2(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btTriangleShapeEx_new3(...)
		return self
	end
	function library.CreateTriangleShapeEx3(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btTriangleShapeEx_new(...)
		return self
	end
	function META:BuildTriPlane(...)
		return CLIB.btTriangleShapeEx_buildTriPlane(self.ptr, ...)
	end
	function META:ApplyTransform(...)
		return CLIB.btTriangleShapeEx_applyTransform(self.ptr, ...)
	end
end
do -- btSoftBodyHelpers
	local META = {}
	library.metatables.btSoftBodyHelpers = META
	META.__index = function(s, k) return META[k] end
	function META:DrawFrame(...)
		return CLIB.btSoftBodyHelpers_DrawFrame(self.ptr, ...)
	end
	function META:Draw(...)
		return CLIB.btSoftBodyHelpers_Draw(self.ptr, ...)
	end
	function META:DrawNodeTree(...)
		return CLIB.btSoftBodyHelpers_DrawNodeTree(self.ptr, ...)
	end
	function META:DrawInfos(...)
		return CLIB.btSoftBodyHelpers_DrawInfos(self.ptr, ...)
	end
	function META:DrawClusterTree(...)
		return CLIB.btSoftBodyHelpers_DrawClusterTree(self.ptr, ...)
	end
	function META:CreatePatchUV(...)
		return CLIB.btSoftBodyHelpers_CreatePatchUV(self.ptr, ...)
	end
	function META:DrawFaceTree(...)
		return CLIB.btSoftBodyHelpers_DrawFaceTree(self.ptr, ...)
	end
	function META:CreateFromConvexHull(...)
		return CLIB.btSoftBodyHelpers_CreateFromConvexHull(self.ptr, ...)
	end
end
do -- btSoftBody_Config
	local META = {}
	library.metatables.btSoftBody_Config = META
	META.__index = function(s, k) return META[k] end
	function META:GetMaxvolume(...)
		return CLIB.btSoftBody_Config_getMaxvolume(self.ptr, ...)
	end
	function META:GetKDG(...)
		return CLIB.btSoftBody_Config_getKDG(self.ptr, ...)
	end
	function META:SetKKHR(...)
		return CLIB.btSoftBody_Config_setKKHR(self.ptr, ...)
	end
	function META:GetKPR(...)
		return CLIB.btSoftBody_Config_getKPR(self.ptr, ...)
	end
	function META:GetDsequence(...)
		return CLIB.btSoftBody_Config_getDsequence(self.ptr, ...)
	end
	function META:GetKDF(...)
		return CLIB.btSoftBody_Config_getKDF(self.ptr, ...)
	end
	function META:GetCollisions(...)
		return CLIB.btSoftBody_Config_getCollisions(self.ptr, ...)
	end
	function META:SetKLF(...)
		return CLIB.btSoftBody_Config_setKLF(self.ptr, ...)
	end
	function META:GetKVC(...)
		return CLIB.btSoftBody_Config_getKVC(self.ptr, ...)
	end
	function META:SetKDG(...)
		return CLIB.btSoftBody_Config_setKDG(self.ptr, ...)
	end
	function META:SetViterations(...)
		return CLIB.btSoftBody_Config_setViterations(self.ptr, ...)
	end
	function META:SetTimescale(...)
		return CLIB.btSoftBody_Config_setTimescale(self.ptr, ...)
	end
	function META:SetPiterations(...)
		return CLIB.btSoftBody_Config_setPiterations(self.ptr, ...)
	end
	function META:SetMaxvolume(...)
		return CLIB.btSoftBody_Config_setMaxvolume(self.ptr, ...)
	end
	function META:SetKVCF(...)
		return CLIB.btSoftBody_Config_setKVCF(self.ptr, ...)
	end
	function META:SetKVC(...)
		return CLIB.btSoftBody_Config_setKVC(self.ptr, ...)
	end
	function META:GetKCHR(...)
		return CLIB.btSoftBody_Config_getKCHR(self.ptr, ...)
	end
	function META:SetKSHR(...)
		return CLIB.btSoftBody_Config_setKSHR(self.ptr, ...)
	end
	function META:SetKPR(...)
		return CLIB.btSoftBody_Config_setKPR(self.ptr, ...)
	end
	function META:SetKMT(...)
		return CLIB.btSoftBody_Config_setKMT(self.ptr, ...)
	end
	function META:SetKDP(...)
		return CLIB.btSoftBody_Config_setKDP(self.ptr, ...)
	end
	function META:SetKDF(...)
		return CLIB.btSoftBody_Config_setKDF(self.ptr, ...)
	end
	function META:SetKCHR(...)
		return CLIB.btSoftBody_Config_setKCHR(self.ptr, ...)
	end
	function META:SetKAHR(...)
		return CLIB.btSoftBody_Config_setKAHR(self.ptr, ...)
	end
	function META:SetDiterations(...)
		return CLIB.btSoftBody_Config_setDiterations(self.ptr, ...)
	end
	function META:SetCollisions(...)
		return CLIB.btSoftBody_Config_setCollisions(self.ptr, ...)
	end
	function META:SetCiterations(...)
		return CLIB.btSoftBody_Config_setCiterations(self.ptr, ...)
	end
	function META:SetAeromodel(...)
		return CLIB.btSoftBody_Config_setAeromodel(self.ptr, ...)
	end
	function META:GetVsequence(...)
		return CLIB.btSoftBody_Config_getVsequence(self.ptr, ...)
	end
	function META:GetViterations(...)
		return CLIB.btSoftBody_Config_getViterations(self.ptr, ...)
	end
	function META:GetPsequence(...)
		return CLIB.btSoftBody_Config_getPsequence(self.ptr, ...)
	end
	function META:GetPiterations(...)
		return CLIB.btSoftBody_Config_getPiterations(self.ptr, ...)
	end
	function META:GetKVCF(...)
		return CLIB.btSoftBody_Config_getKVCF(self.ptr, ...)
	end
	function META:GetKSHR(...)
		return CLIB.btSoftBody_Config_getKSHR(self.ptr, ...)
	end
	function META:GetKMT(...)
		return CLIB.btSoftBody_Config_getKMT(self.ptr, ...)
	end
	function META:GetKLF(...)
		return CLIB.btSoftBody_Config_getKLF(self.ptr, ...)
	end
	function META:GetKKHR(...)
		return CLIB.btSoftBody_Config_getKKHR(self.ptr, ...)
	end
	function META:GetKAHR(...)
		return CLIB.btSoftBody_Config_getKAHR(self.ptr, ...)
	end
	function META:GetDiterations(...)
		return CLIB.btSoftBody_Config_getDiterations(self.ptr, ...)
	end
	function META:GetTimescale(...)
		return CLIB.btSoftBody_Config_getTimescale(self.ptr, ...)
	end
	function META:GetKDP(...)
		return CLIB.btSoftBody_Config_getKDP(self.ptr, ...)
	end
	function META:GetCiterations(...)
		return CLIB.btSoftBody_Config_getCiterations(self.ptr, ...)
	end
	function META:GetAeromodel(...)
		return CLIB.btSoftBody_Config_getAeromodel(self.ptr, ...)
	end
end
do -- btSoftBody_CJoint
	local META = {}
	library.metatables.btSoftBody_CJoint = META
	META.__index = function(s, k) return META[k] end
	function META:GetMaxlife(...)
		return CLIB.btSoftBody_CJoint_getMaxlife(self.ptr, ...)
	end
	function META:SetNormal(...)
		return CLIB.btSoftBody_CJoint_setNormal(self.ptr, ...)
	end
	function META:SetMaxlife(...)
		return CLIB.btSoftBody_CJoint_setMaxlife(self.ptr, ...)
	end
	function META:GetLife(...)
		return CLIB.btSoftBody_CJoint_getLife(self.ptr, ...)
	end
	function META:GetRpos(...)
		return CLIB.btSoftBody_CJoint_getRpos(self.ptr, ...)
	end
	function META:SetLife(...)
		return CLIB.btSoftBody_CJoint_setLife(self.ptr, ...)
	end
	function META:SetFriction(...)
		return CLIB.btSoftBody_CJoint_setFriction(self.ptr, ...)
	end
	function META:GetNormal(...)
		return CLIB.btSoftBody_CJoint_getNormal(self.ptr, ...)
	end
	function META:GetFriction(...)
		return CLIB.btSoftBody_CJoint_getFriction(self.ptr, ...)
	end
end
do -- btTriangleInfoMap
	local META = {}
	library.metatables.btTriangleInfoMap = META
	META.__index = function(s, k) return META[k] end
	function library.CreateTriangleInfoMap(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btTriangleInfoMap_new(...)
		return self
	end
	function META:GetConvexEpsilon(...)
		return CLIB.btTriangleInfoMap_getConvexEpsilon(self.ptr, ...)
	end
	function META:GetMaxEdgeAngleThreshold(...)
		return CLIB.btTriangleInfoMap_getMaxEdgeAngleThreshold(self.ptr, ...)
	end
	function META:SetEdgeDistanceThreshold(...)
		return CLIB.btTriangleInfoMap_setEdgeDistanceThreshold(self.ptr, ...)
	end
	function META:GetEdgeDistanceThreshold(...)
		return CLIB.btTriangleInfoMap_getEdgeDistanceThreshold(self.ptr, ...)
	end
	function META:SetPlanarEpsilon(...)
		return CLIB.btTriangleInfoMap_setPlanarEpsilon(self.ptr, ...)
	end
	function META:SetMaxEdgeAngleThreshold(...)
		return CLIB.btTriangleInfoMap_setMaxEdgeAngleThreshold(self.ptr, ...)
	end
	function META:SetConvexEpsilon(...)
		return CLIB.btTriangleInfoMap_setConvexEpsilon(self.ptr, ...)
	end
	function META:Serialize(...)
		return CLIB.btTriangleInfoMap_serialize(self.ptr, ...)
	end
	function META:GetPlanarEpsilon(...)
		return CLIB.btTriangleInfoMap_getPlanarEpsilon(self.ptr, ...)
	end
	function META:GetEqualVertexThreshold(...)
		return CLIB.btTriangleInfoMap_getEqualVertexThreshold(self.ptr, ...)
	end
	function META:CalculateSerializeBufferSize(...)
		return CLIB.btTriangleInfoMap_calculateSerializeBufferSize(self.ptr, ...)
	end
	function META:Delete(...)
		return CLIB.btTriangleInfoMap_delete(self.ptr, ...)
	end
	function META:SetZeroAreaThreshold(...)
		return CLIB.btTriangleInfoMap_setZeroAreaThreshold(self.ptr, ...)
	end
	function META:GetZeroAreaThreshold(...)
		return CLIB.btTriangleInfoMap_getZeroAreaThreshold(self.ptr, ...)
	end
	function META:SetEqualVertexThreshold(...)
		return CLIB.btTriangleInfoMap_setEqualVertexThreshold(self.ptr, ...)
	end
end
do -- btActionInterface
	local META = {}
	library.metatables.btActionInterface = META
	META.__index = function(s, k) return META[k] end
	function META:Delete(...)
		return CLIB.btActionInterface_delete(self.ptr, ...)
	end
	function META:UpdateAction(...)
		return CLIB.btActionInterface_updateAction(self.ptr, ...)
	end
	function META:DebugDraw(...)
		return CLIB.btActionInterface_debugDraw(self.ptr, ...)
	end
end
do -- btSoftBody_LJoint
	local META = {}
	library.metatables.btSoftBody_LJoint = META
	META.__index = function(s, k) return META[k] end
	function META:GetRpos(...)
		return CLIB.btSoftBody_LJoint_getRpos(self.ptr, ...)
	end
end
do -- btCollisionObject
	local META = {}
	library.metatables.btCollisionObject = META
	META.__index = function(s, k) return META[k] end
	function library.CreateCollisionObject(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btCollisionObject_new(...)
		return self
	end
	function META:SetWorldArrayIndex(...)
		return CLIB.btCollisionObject_setWorldArrayIndex(self.ptr, ...)
	end
	function META:IsActive(...)
		return CLIB.btCollisionObject_isActive(self.ptr, ...)
	end
	function META:SetActivationState(...)
		return CLIB.btCollisionObject_setActivationState(self.ptr, ...)
	end
	function META:SetAnisotropicFriction(...)
		return CLIB.btCollisionObject_setAnisotropicFriction(self.ptr, ...)
	end
	function META:GetCollisionFlags(...)
		return CLIB.btCollisionObject_getCollisionFlags(self.ptr, ...)
	end
	function META:SetInterpolationAngularVelocity(...)
		return CLIB.btCollisionObject_setInterpolationAngularVelocity(self.ptr, ...)
	end
	function META:InternalSetExtensionPointer(...)
		return CLIB.btCollisionObject_internalSetExtensionPointer(self.ptr, ...)
	end
	function META:SetCompanionId(...)
		return CLIB.btCollisionObject_setCompanionId(self.ptr, ...)
	end
	function META:IsKinematicObject(...)
		return CLIB.btCollisionObject_isKinematicObject(self.ptr, ...)
	end
	function META:SerializeSingleObject(...)
		return CLIB.btCollisionObject_serializeSingleObject(self.ptr, ...)
	end
	function META:SetContactStiffnessAndDamping(...)
		return CLIB.btCollisionObject_setContactStiffnessAndDamping(self.ptr, ...)
	end
	function META:GetCompanionId(...)
		return CLIB.btCollisionObject_getCompanionId(self.ptr, ...)
	end
	function META:Serialize(...)
		return CLIB.btCollisionObject_serialize(self.ptr, ...)
	end
	function META:SetDeactivationTime(...)
		return CLIB.btCollisionObject_setDeactivationTime(self.ptr, ...)
	end
	function META:SetInterpolationWorldTransform(...)
		return CLIB.btCollisionObject_setInterpolationWorldTransform(self.ptr, ...)
	end
	function META:GetWorldTransform(...)
		return CLIB.btCollisionObject_getWorldTransform(self.ptr, ...)
	end
	function META:GetInterpolationLinearVelocity(...)
		return CLIB.btCollisionObject_getInterpolationLinearVelocity(self.ptr, ...)
	end
	function META:IsStaticObject(...)
		return CLIB.btCollisionObject_isStaticObject(self.ptr, ...)
	end
	function META:SetCollisionShape(...)
		return CLIB.btCollisionObject_setCollisionShape(self.ptr, ...)
	end
	function META:CalculateSerializeBufferSize(...)
		return CLIB.btCollisionObject_calculateSerializeBufferSize(self.ptr, ...)
	end
	function META:IsStaticOrKinematicObject(...)
		return CLIB.btCollisionObject_isStaticOrKinematicObject(self.ptr, ...)
	end
	function META:GetContactStiffness(...)
		return CLIB.btCollisionObject_getContactStiffness(self.ptr, ...)
	end
	function META:GetBroadphaseHandle(...)
		return CLIB.btCollisionObject_getBroadphaseHandle(self.ptr, ...)
	end
	function META:SetCcdSweptSphereRadius(...)
		return CLIB.btCollisionObject_setCcdSweptSphereRadius(self.ptr, ...)
	end
	function META:SetContactProcessingThreshold(...)
		return CLIB.btCollisionObject_setContactProcessingThreshold(self.ptr, ...)
	end
	function META:CheckCollideWith(...)
		return CLIB.btCollisionObject_checkCollideWith(self.ptr, ...)
	end
	function META:GetContactProcessingThreshold(...)
		return CLIB.btCollisionObject_getContactProcessingThreshold(self.ptr, ...)
	end
	function META:SetUserPointer(...)
		return CLIB.btCollisionObject_setUserPointer(self.ptr, ...)
	end
	function META:SetUserIndex(...)
		return CLIB.btCollisionObject_setUserIndex(self.ptr, ...)
	end
	function META:SetFriction(...)
		return CLIB.btCollisionObject_setFriction(self.ptr, ...)
	end
	function META:GetCollisionShape(...)
		return CLIB.btCollisionObject_getCollisionShape(self.ptr, ...)
	end
	function META:SetSpinningFriction(...)
		return CLIB.btCollisionObject_setSpinningFriction(self.ptr, ...)
	end
	function META:GetUserIndex(...)
		return CLIB.btCollisionObject_getUserIndex(self.ptr, ...)
	end
	function META:GetAnisotropicFriction(...)
		return CLIB.btCollisionObject_getAnisotropicFriction(self.ptr, ...)
	end
	function META:RemoveCustomDebugColor(...)
		return CLIB.btCollisionObject_removeCustomDebugColor(self.ptr, ...)
	end
	function META:SetWorldTransform(...)
		return CLIB.btCollisionObject_setWorldTransform(self.ptr, ...)
	end
	function META:SetIgnoreCollisionCheck(...)
		return CLIB.btCollisionObject_setIgnoreCollisionCheck(self.ptr, ...)
	end
	function META:GetCcdSweptSphereRadius(...)
		return CLIB.btCollisionObject_getCcdSweptSphereRadius(self.ptr, ...)
	end
	function META:ForceActivationState(...)
		return CLIB.btCollisionObject_forceActivationState(self.ptr, ...)
	end
	function META:CheckCollideWithOverride(...)
		return CLIB.btCollisionObject_checkCollideWithOverride(self.ptr, ...)
	end
	function META:GetRollingFriction(...)
		return CLIB.btCollisionObject_getRollingFriction(self.ptr, ...)
	end
	function META:HasContactResponse(...)
		return CLIB.btCollisionObject_hasContactResponse(self.ptr, ...)
	end
	function META:GetSpinningFriction(...)
		return CLIB.btCollisionObject_getSpinningFriction(self.ptr, ...)
	end
	function META:GetContactDamping(...)
		return CLIB.btCollisionObject_getContactDamping(self.ptr, ...)
	end
	function META:SetIslandTag(...)
		return CLIB.btCollisionObject_setIslandTag(self.ptr, ...)
	end
	function META:SetHitFraction(...)
		return CLIB.btCollisionObject_setHitFraction(self.ptr, ...)
	end
	function META:SetBroadphaseHandle(...)
		return CLIB.btCollisionObject_setBroadphaseHandle(self.ptr, ...)
	end
	function META:Delete(...)
		return CLIB.btCollisionObject_delete(self.ptr, ...)
	end
	function META:GetCcdSquareMotionThreshold(...)
		return CLIB.btCollisionObject_getCcdSquareMotionThreshold(self.ptr, ...)
	end
	function META:SetRollingFriction(...)
		return CLIB.btCollisionObject_setRollingFriction(self.ptr, ...)
	end
	function META:GetWorldArrayIndex(...)
		return CLIB.btCollisionObject_getWorldArrayIndex(self.ptr, ...)
	end
	function META:GetDeactivationTime(...)
		return CLIB.btCollisionObject_getDeactivationTime(self.ptr, ...)
	end
	function META:GetInterpolationAngularVelocity(...)
		return CLIB.btCollisionObject_getInterpolationAngularVelocity(self.ptr, ...)
	end
	function META:Activate(...)
		return CLIB.btCollisionObject_activate(self.ptr, ...)
	end
	function META:SetCustomDebugColor(...)
		return CLIB.btCollisionObject_setCustomDebugColor(self.ptr, ...)
	end
	function META:GetActivationState(...)
		return CLIB.btCollisionObject_getActivationState(self.ptr, ...)
	end
	function META:MergesSimulationIslands(...)
		return CLIB.btCollisionObject_mergesSimulationIslands(self.ptr, ...)
	end
	function META:GetInterpolationWorldTransform(...)
		return CLIB.btCollisionObject_getInterpolationWorldTransform(self.ptr, ...)
	end
	function META:GetCustomDebugColor(...)
		return CLIB.btCollisionObject_getCustomDebugColor(self.ptr, ...)
	end
	function META:GetIslandTag(...)
		return CLIB.btCollisionObject_getIslandTag(self.ptr, ...)
	end
	function META:GetRestitution(...)
		return CLIB.btCollisionObject_getRestitution(self.ptr, ...)
	end
	function META:GetUserIndex2(...)
		return CLIB.btCollisionObject_getUserIndex2(self.ptr, ...)
	end
	function META:GetUserPointer(...)
		return CLIB.btCollisionObject_getUserPointer(self.ptr, ...)
	end
	function META:GetFriction(...)
		return CLIB.btCollisionObject_getFriction(self.ptr, ...)
	end
	function META:SetUserIndex2(...)
		return CLIB.btCollisionObject_setUserIndex2(self.ptr, ...)
	end
	function META:SetRestitution(...)
		return CLIB.btCollisionObject_setRestitution(self.ptr, ...)
	end
	function META:SetInterpolationLinearVelocity(...)
		return CLIB.btCollisionObject_setInterpolationLinearVelocity(self.ptr, ...)
	end
	function META:HasAnisotropicFriction(...)
		return CLIB.btCollisionObject_hasAnisotropicFriction(self.ptr, ...)
	end
	function META:SetCcdMotionThreshold(...)
		return CLIB.btCollisionObject_setCcdMotionThreshold(self.ptr, ...)
	end
	function META:GetCcdMotionThreshold(...)
		return CLIB.btCollisionObject_getCcdMotionThreshold(self.ptr, ...)
	end
	function META:InternalGetExtensionPointer(...)
		return CLIB.btCollisionObject_internalGetExtensionPointer(self.ptr, ...)
	end
	function META:SetCollisionFlags(...)
		return CLIB.btCollisionObject_setCollisionFlags(self.ptr, ...)
	end
	function META:GetHitFraction(...)
		return CLIB.btCollisionObject_getHitFraction(self.ptr, ...)
	end
	function META:GetInternalType(...)
		return CLIB.btCollisionObject_getInternalType(self.ptr, ...)
	end
end
do -- btAABB_get_center
	local META = {}
	library.metatables.btAABB_get_center = META
	META.__index = function(s, k) return META[k] end
	function META:Extend(...)
		return CLIB.btAABB_get_center_extend(self.ptr, ...)
	end
end
do -- btSoftBody_PSolve
	local META = {}
	library.metatables.btSoftBody_PSolve = META
	META.__index = function(s, k) return META[k] end
	function META:Anchors(...)
		return CLIB.btSoftBody_PSolve_Anchors(self.ptr, ...)
	end
	function META:SContacts(...)
		return CLIB.btSoftBody_PSolve_SContacts(self.ptr, ...)
	end
	function META:RContacts(...)
		return CLIB.btSoftBody_PSolve_RContacts(self.ptr, ...)
	end
	function META:Links(...)
		return CLIB.btSoftBody_PSolve_Links(self.ptr, ...)
	end
end
do -- btGImpactBvh_find
	local META = {}
	library.metatables.btGImpactBvh_find = META
	META.__index = function(s, k) return META[k] end
	function META:Collision(...)
		return CLIB.btGImpactBvh_find_collision(self.ptr, ...)
	end
end
do -- btSoftBody_Anchor
	local META = {}
	library.metatables.btSoftBody_Anchor = META
	META.__index = function(s, k) return META[k] end
	function META:GetInfluence(...)
		return CLIB.btSoftBody_Anchor_getInfluence(self.ptr, ...)
	end
	function META:GetLocal(...)
		return CLIB.btSoftBody_Anchor_getLocal(self.ptr, ...)
	end
	function META:GetNode(...)
		return CLIB.btSoftBody_Anchor_getNode(self.ptr, ...)
	end
	function META:SetC2(...)
		return CLIB.btSoftBody_Anchor_setC2(self.ptr, ...)
	end
	function META:SetNode(...)
		return CLIB.btSoftBody_Anchor_setNode(self.ptr, ...)
	end
	function META:SetLocal(...)
		return CLIB.btSoftBody_Anchor_setLocal(self.ptr, ...)
	end
	function META:SetInfluence(...)
		return CLIB.btSoftBody_Anchor_setInfluence(self.ptr, ...)
	end
	function META:SetC1(...)
		return CLIB.btSoftBody_Anchor_setC1(self.ptr, ...)
	end
	function META:SetC0(...)
		return CLIB.btSoftBody_Anchor_setC0(self.ptr, ...)
	end
	function META:SetBody(...)
		return CLIB.btSoftBody_Anchor_setBody(self.ptr, ...)
	end
	function META:GetC2(...)
		return CLIB.btSoftBody_Anchor_getC2(self.ptr, ...)
	end
	function META:GetC1(...)
		return CLIB.btSoftBody_Anchor_getC1(self.ptr, ...)
	end
	function META:GetC0(...)
		return CLIB.btSoftBody_Anchor_getC0(self.ptr, ...)
	end
	function META:GetBody(...)
		return CLIB.btSoftBody_Anchor_getBody(self.ptr, ...)
	end
end
do -- btFixedConstraint
	local META = {}
	library.metatables.btFixedConstraint = META
	META.__index = function(s, k) return META[k] end
	function library.CreateFixedConstraint(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btFixedConstraint_new(...)
		return self
	end
end
do -- btCollisionWorld
	local META = {}
	library.metatables.btCollisionWorld = META
	META.__index = function(s, k) return META[k] end
	function library.CreateCollisionWorld(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btCollisionWorld_new(...)
		return self
	end
	function META:ContactPairTest(...)
		return CLIB.btCollisionWorld_contactPairTest(self.ptr, ...)
	end
	function META:Delete(...)
		return CLIB.btCollisionWorld_delete(self.ptr, ...)
	end
	function META:SetBroadphase(...)
		return CLIB.btCollisionWorld_setBroadphase(self.ptr, ...)
	end
	function META:RayTest(...)
		return CLIB.btCollisionWorld_rayTest(self.ptr, ...)
	end
	function META:AddCollisionObject2(...)
		return CLIB.btCollisionWorld_addCollisionObject2(self.ptr, ...)
	end
	function META:SetForceUpdateAllAabbs(...)
		return CLIB.btCollisionWorld_setForceUpdateAllAabbs(self.ptr, ...)
	end
	function META:SetDebugDrawer(...)
		return CLIB.btCollisionWorld_setDebugDrawer(self.ptr, ...)
	end
	function META:DebugDrawObject(...)
		return CLIB.btCollisionWorld_debugDrawObject(self.ptr, ...)
	end
	function META:UpdateAabbs(...)
		return CLIB.btCollisionWorld_updateAabbs(self.ptr, ...)
	end
	function META:GetNumCollisionObjects(...)
		return CLIB.btCollisionWorld_getNumCollisionObjects(self.ptr, ...)
	end
	function META:RemoveCollisionObject(...)
		return CLIB.btCollisionWorld_removeCollisionObject(self.ptr, ...)
	end
	function META:RayTestSingle(...)
		return CLIB.btCollisionWorld_rayTestSingle(self.ptr, ...)
	end
	function META:GetDispatcher(...)
		return CLIB.btCollisionWorld_getDispatcher(self.ptr, ...)
	end
	function META:GetDebugDrawer(...)
		return CLIB.btCollisionWorld_getDebugDrawer(self.ptr, ...)
	end
	function META:GetDispatchInfo(...)
		return CLIB.btCollisionWorld_getDispatchInfo(self.ptr, ...)
	end
	function META:UpdateSingleAabb(...)
		return CLIB.btCollisionWorld_updateSingleAabb(self.ptr, ...)
	end
	function META:GetPairCache(...)
		return CLIB.btCollisionWorld_getPairCache(self.ptr, ...)
	end
	function META:ComputeOverlappingPairs(...)
		return CLIB.btCollisionWorld_computeOverlappingPairs(self.ptr, ...)
	end
	function META:ObjectQuerySingleInternal(...)
		return CLIB.btCollisionWorld_objectQuerySingleInternal(self.ptr, ...)
	end
	function META:GetCollisionObjectArray(...)
		return CLIB.btCollisionWorld_getCollisionObjectArray(self.ptr, ...)
	end
	function META:ConvexSweepTest(...)
		return CLIB.btCollisionWorld_convexSweepTest(self.ptr, ...)
	end
	function META:ContactTest(...)
		return CLIB.btCollisionWorld_contactTest(self.ptr, ...)
	end
	function META:ObjectQuerySingle(...)
		return CLIB.btCollisionWorld_objectQuerySingle(self.ptr, ...)
	end
	function META:AddCollisionObject(...)
		return CLIB.btCollisionWorld_addCollisionObject(self.ptr, ...)
	end
	function META:Serialize(...)
		return CLIB.btCollisionWorld_serialize(self.ptr, ...)
	end
	function META:AddCollisionObject3(...)
		return CLIB.btCollisionWorld_addCollisionObject3(self.ptr, ...)
	end
	function META:PerformDiscreteCollisionDetection(...)
		return CLIB.btCollisionWorld_performDiscreteCollisionDetection(self.ptr, ...)
	end
	function META:GetForceUpdateAllAabbs(...)
		return CLIB.btCollisionWorld_getForceUpdateAllAabbs(self.ptr, ...)
	end
	function META:GetBroadphase(...)
		return CLIB.btCollisionWorld_getBroadphase(self.ptr, ...)
	end
	function META:DebugDrawWorld(...)
		return CLIB.btCollisionWorld_debugDrawWorld(self.ptr, ...)
	end
	function META:RayTestSingleInternal(...)
		return CLIB.btCollisionWorld_rayTestSingleInternal(self.ptr, ...)
	end
end
do -- btCylinderShapeZ
	local META = {}
	library.metatables.btCylinderShapeZ = META
	META.__index = function(s, k) return META[k] end
	function library.CreateCylinderShapeZ(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btCylinderShapeZ_new2(...)
		return self
	end
	function library.CreateCylinderShapeZ2(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btCylinderShapeZ_new(...)
		return self
	end
end
do -- btBU_Simplex1to4
	local META = {}
	library.metatables.btBU_Simplex1to4 = META
	META.__index = function(s, k) return META[k] end
	function library.CreateBU_Simplex1to4(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btBU_Simplex1to4_new(...)
		return self
	end
	function library.CreateBU_Simplex1to42(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btBU_Simplex1to4_new4(...)
		return self
	end
	function library.CreateBU_Simplex1to43(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btBU_Simplex1to4_new5(...)
		return self
	end
	function library.CreateBU_Simplex1to44(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btBU_Simplex1to4_new3(...)
		return self
	end
	function library.CreateBU_Simplex1to45(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btBU_Simplex1to4_new2(...)
		return self
	end
	function META:GetIndex(...)
		return CLIB.btBU_Simplex1to4_getIndex(self.ptr, ...)
	end
	function META:Reset(...)
		return CLIB.btBU_Simplex1to4_reset(self.ptr, ...)
	end
	function META:AddVertex(...)
		return CLIB.btBU_Simplex1to4_addVertex(self.ptr, ...)
	end
end
do -- btBoxBoxDetector
	local META = {}
	library.metatables.btBoxBoxDetector = META
	META.__index = function(s, k) return META[k] end
	function library.CreateBoxBoxDetector(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btBoxBoxDetector_new(...)
		return self
	end
	function META:GetBox1(...)
		return CLIB.btBoxBoxDetector_getBox1(self.ptr, ...)
	end
	function META:GetBox2(...)
		return CLIB.btBoxBoxDetector_getBox2(self.ptr, ...)
	end
	function META:SetBox1(...)
		return CLIB.btBoxBoxDetector_setBox1(self.ptr, ...)
	end
	function META:SetBox2(...)
		return CLIB.btBoxBoxDetector_setBox2(self.ptr, ...)
	end
end
do -- btBroadphasePair
	local META = {}
	library.metatables.btBroadphasePair = META
	META.__index = function(s, k) return META[k] end
	function library.CreateBroadphasePair(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btBroadphasePair_new(...)
		return self
	end
	function library.CreateBroadphasePair2(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btBroadphasePair_new2(...)
		return self
	end
	function library.CreateBroadphasePair3(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btBroadphasePair_new3(...)
		return self
	end
	function META:SetPProxy0(...)
		return CLIB.btBroadphasePair_setPProxy0(self.ptr, ...)
	end
	function META:Delete(...)
		return CLIB.btBroadphasePair_delete(self.ptr, ...)
	end
	function META:SetPProxy1(...)
		return CLIB.btBroadphasePair_setPProxy1(self.ptr, ...)
	end
	function META:SetAlgorithm(...)
		return CLIB.btBroadphasePair_setAlgorithm(self.ptr, ...)
	end
	function META:GetAlgorithm(...)
		return CLIB.btBroadphasePair_getAlgorithm(self.ptr, ...)
	end
	function META:GetPProxy1(...)
		return CLIB.btBroadphasePair_getPProxy1(self.ptr, ...)
	end
	function META:GetPProxy0(...)
		return CLIB.btBroadphasePair_getPProxy0(self.ptr, ...)
	end
end
do -- btAABB_copy_with
	local META = {}
	library.metatables.btAABB_copy_with = META
	META.__index = function(s, k) return META[k] end
	function META:Margin(...)
		return CLIB.btAABB_copy_with_margin(self.ptr, ...)
	end
end
do -- btRaycastVehicle
	local META = {}
	library.metatables.btRaycastVehicle = META
	META.__index = function(s, k) return META[k] end
	function library.CreateRaycastVehicle(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btRaycastVehicle_new(...)
		return self
	end
	function META:GetSteeringValue(...)
		return CLIB.btRaycastVehicle_getSteeringValue(self.ptr, ...)
	end
	function META:SetPitchControl(...)
		return CLIB.btRaycastVehicle_setPitchControl(self.ptr, ...)
	end
	function META:GetForwardVector(...)
		return CLIB.btRaycastVehicle_getForwardVector(self.ptr, ...)
	end
	function META:SetBrake(...)
		return CLIB.btRaycastVehicle_setBrake(self.ptr, ...)
	end
	function META:SetCoordinateSystem(...)
		return CLIB.btRaycastVehicle_setCoordinateSystem(self.ptr, ...)
	end
	function META:GetCurrentSpeedKmHour(...)
		return CLIB.btRaycastVehicle_getCurrentSpeedKmHour(self.ptr, ...)
	end
	function META:GetUpAxis(...)
		return CLIB.btRaycastVehicle_getUpAxis(self.ptr, ...)
	end
	function META:UpdateWheelTransformsWS(...)
		return CLIB.btRaycastVehicle_updateWheelTransformsWS(self.ptr, ...)
	end
	function META:SetSteeringValue(...)
		return CLIB.btRaycastVehicle_setSteeringValue(self.ptr, ...)
	end
	function META:SetUserConstraintType(...)
		return CLIB.btRaycastVehicle_setUserConstraintType(self.ptr, ...)
	end
	function META:GetRigidBody(...)
		return CLIB.btRaycastVehicle_getRigidBody(self.ptr, ...)
	end
	function META:UpdateWheelTransformsWS2(...)
		return CLIB.btRaycastVehicle_updateWheelTransformsWS2(self.ptr, ...)
	end
	function META:SetUserConstraintId(...)
		return CLIB.btRaycastVehicle_setUserConstraintId(self.ptr, ...)
	end
	function META:GetChassisWorldTransform(...)
		return CLIB.btRaycastVehicle_getChassisWorldTransform(self.ptr, ...)
	end
	function META:GetForwardAxis(...)
		return CLIB.btRaycastVehicle_getForwardAxis(self.ptr, ...)
	end
	function META:UpdateFriction(...)
		return CLIB.btRaycastVehicle_updateFriction(self.ptr, ...)
	end
	function META:UpdateWheelTransform2(...)
		return CLIB.btRaycastVehicle_updateWheelTransform2(self.ptr, ...)
	end
	function META:UpdateWheelTransform(...)
		return CLIB.btRaycastVehicle_updateWheelTransform(self.ptr, ...)
	end
	function META:UpdateVehicle(...)
		return CLIB.btRaycastVehicle_updateVehicle(self.ptr, ...)
	end
	function META:UpdateSuspension(...)
		return CLIB.btRaycastVehicle_updateSuspension(self.ptr, ...)
	end
	function META:RayCast(...)
		return CLIB.btRaycastVehicle_rayCast(self.ptr, ...)
	end
	function META:GetWheelInfo2(...)
		return CLIB.btRaycastVehicle_getWheelInfo2(self.ptr, ...)
	end
	function META:GetWheelInfo(...)
		return CLIB.btRaycastVehicle_getWheelInfo(self.ptr, ...)
	end
	function META:GetUserConstraintType(...)
		return CLIB.btRaycastVehicle_getUserConstraintType(self.ptr, ...)
	end
	function META:GetRightAxis(...)
		return CLIB.btRaycastVehicle_getRightAxis(self.ptr, ...)
	end
	function META:GetNumWheels(...)
		return CLIB.btRaycastVehicle_getNumWheels(self.ptr, ...)
	end
	function META:ApplyEngineForce(...)
		return CLIB.btRaycastVehicle_applyEngineForce(self.ptr, ...)
	end
	function META:GetUserConstraintId(...)
		return CLIB.btRaycastVehicle_getUserConstraintId(self.ptr, ...)
	end
	function META:AddWheel(...)
		return CLIB.btRaycastVehicle_addWheel(self.ptr, ...)
	end
	function META:GetWheelTransformWS(...)
		return CLIB.btRaycastVehicle_getWheelTransformWS(self.ptr, ...)
	end
	function META:ResetSuspension(...)
		return CLIB.btRaycastVehicle_resetSuspension(self.ptr, ...)
	end
end
do -- btDispatcherInfo
	local META = {}
	library.metatables.btDispatcherInfo = META
	META.__index = function(s, k) return META[k] end
	function META:SetTimeOfImpact(...)
		return CLIB.btDispatcherInfo_setTimeOfImpact(self.ptr, ...)
	end
	function META:GetTimeStep(...)
		return CLIB.btDispatcherInfo_getTimeStep(self.ptr, ...)
	end
	function META:GetStepCount(...)
		return CLIB.btDispatcherInfo_getStepCount(self.ptr, ...)
	end
	function META:SetTimeStep(...)
		return CLIB.btDispatcherInfo_setTimeStep(self.ptr, ...)
	end
	function META:SetDispatchFunc(...)
		return CLIB.btDispatcherInfo_setDispatchFunc(self.ptr, ...)
	end
	function META:GetUseEpa(...)
		return CLIB.btDispatcherInfo_getUseEpa(self.ptr, ...)
	end
	function META:GetEnableSPU(...)
		return CLIB.btDispatcherInfo_getEnableSPU(self.ptr, ...)
	end
	function META:SetUseContinuous(...)
		return CLIB.btDispatcherInfo_setUseContinuous(self.ptr, ...)
	end
	function META:GetEnableSatConvex(...)
		return CLIB.btDispatcherInfo_getEnableSatConvex(self.ptr, ...)
	end
	function META:GetTimeOfImpact(...)
		return CLIB.btDispatcherInfo_getTimeOfImpact(self.ptr, ...)
	end
	function META:GetDebugDraw(...)
		return CLIB.btDispatcherInfo_getDebugDraw(self.ptr, ...)
	end
	function META:GetAllowedCcdPenetration(...)
		return CLIB.btDispatcherInfo_getAllowedCcdPenetration(self.ptr, ...)
	end
	function META:SetEnableSatConvex(...)
		return CLIB.btDispatcherInfo_setEnableSatConvex(self.ptr, ...)
	end
	function META:GetUseContinuous(...)
		return CLIB.btDispatcherInfo_getUseContinuous(self.ptr, ...)
	end
	function META:SetDebugDraw(...)
		return CLIB.btDispatcherInfo_setDebugDraw(self.ptr, ...)
	end
	function META:GetDispatchFunc(...)
		return CLIB.btDispatcherInfo_getDispatchFunc(self.ptr, ...)
	end
	function META:SetUseEpa(...)
		return CLIB.btDispatcherInfo_setUseEpa(self.ptr, ...)
	end
	function META:GetConvexConservativeDistanceThreshold(...)
		return CLIB.btDispatcherInfo_getConvexConservativeDistanceThreshold(self.ptr, ...)
	end
	function META:GetUseConvexConservativeDistanceUtil(...)
		return CLIB.btDispatcherInfo_getUseConvexConservativeDistanceUtil(self.ptr, ...)
	end
	function META:SetConvexConservativeDistanceThreshold(...)
		return CLIB.btDispatcherInfo_setConvexConservativeDistanceThreshold(self.ptr, ...)
	end
	function META:SetEnableSPU(...)
		return CLIB.btDispatcherInfo_setEnableSPU(self.ptr, ...)
	end
	function META:SetUseConvexConservativeDistanceUtil(...)
		return CLIB.btDispatcherInfo_setUseConvexConservativeDistanceUtil(self.ptr, ...)
	end
	function META:SetAllowedCcdPenetration(...)
		return CLIB.btDispatcherInfo_setAllowedCcdPenetration(self.ptr, ...)
	end
	function META:SetStepCount(...)
		return CLIB.btDispatcherInfo_setStepCount(self.ptr, ...)
	end
end
do -- btManifoldResult
	local META = {}
	library.metatables.btManifoldResult = META
	META.__index = function(s, k) return META[k] end
	function library.CreateManifoldResult(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btManifoldResult_new2(...)
		return self
	end
	function library.CreateManifoldResult2(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btManifoldResult_new(...)
		return self
	end
	function META:GetBody0Wrap(...)
		return CLIB.btManifoldResult_getBody0Wrap(self.ptr, ...)
	end
	function META:SetPersistentManifold(...)
		return CLIB.btManifoldResult_setPersistentManifold(self.ptr, ...)
	end
	function META:CalculateCombinedContactStiffness(...)
		return CLIB.btManifoldResult_calculateCombinedContactStiffness(self.ptr, ...)
	end
	function META:GetClosestPointDistanceThreshold(...)
		return CLIB.btManifoldResult_getClosestPointDistanceThreshold(self.ptr, ...)
	end
	function META:CalculateCombinedFriction(...)
		return CLIB.btManifoldResult_calculateCombinedFriction(self.ptr, ...)
	end
	function META:CalculateCombinedRestitution(...)
		return CLIB.btManifoldResult_calculateCombinedRestitution(self.ptr, ...)
	end
	function META:SetBody1Wrap(...)
		return CLIB.btManifoldResult_setBody1Wrap(self.ptr, ...)
	end
	function META:GetPersistentManifold(...)
		return CLIB.btManifoldResult_getPersistentManifold(self.ptr, ...)
	end
	function META:CalculateCombinedContactDamping(...)
		return CLIB.btManifoldResult_calculateCombinedContactDamping(self.ptr, ...)
	end
	function META:SetClosestPointDistanceThreshold(...)
		return CLIB.btManifoldResult_setClosestPointDistanceThreshold(self.ptr, ...)
	end
	function META:GetBody0Internal(...)
		return CLIB.btManifoldResult_getBody0Internal(self.ptr, ...)
	end
	function META:RefreshContactPoints(...)
		return CLIB.btManifoldResult_refreshContactPoints(self.ptr, ...)
	end
	function META:CalculateCombinedRollingFriction(...)
		return CLIB.btManifoldResult_calculateCombinedRollingFriction(self.ptr, ...)
	end
	function META:GetBody1Internal(...)
		return CLIB.btManifoldResult_getBody1Internal(self.ptr, ...)
	end
	function META:GetBody1Wrap(...)
		return CLIB.btManifoldResult_getBody1Wrap(self.ptr, ...)
	end
	function META:SetBody0Wrap(...)
		return CLIB.btManifoldResult_setBody0Wrap(self.ptr, ...)
	end
end
do -- btSoftBody_Joint
	local META = {}
	library.metatables.btSoftBody_Joint = META
	META.__index = function(s, k) return META[k] end
	function META:GetDelete(...)
		return CLIB.btSoftBody_Joint_getDelete(self.ptr, ...)
	end
	function META:SetSplit(...)
		return CLIB.btSoftBody_Joint_setSplit(self.ptr, ...)
	end
	function META:Delete(...)
		return CLIB.btSoftBody_Joint_delete(self.ptr, ...)
	end
	function META:Terminate(...)
		return CLIB.btSoftBody_Joint_Terminate(self.ptr, ...)
	end
	function META:GetSplit(...)
		return CLIB.btSoftBody_Joint_getSplit(self.ptr, ...)
	end
	function META:GetErp(...)
		return CLIB.btSoftBody_Joint_getErp(self.ptr, ...)
	end
	function META:Type(...)
		return CLIB.btSoftBody_Joint_Type(self.ptr, ...)
	end
	function META:GetBodies(...)
		return CLIB.btSoftBody_Joint_getBodies(self.ptr, ...)
	end
	function META:GetRefs(...)
		return CLIB.btSoftBody_Joint_getRefs(self.ptr, ...)
	end
	function META:Solve(...)
		return CLIB.btSoftBody_Joint_Solve(self.ptr, ...)
	end
	function META:SetSdrift(...)
		return CLIB.btSoftBody_Joint_setSdrift(self.ptr, ...)
	end
	function META:SetMassmatrix(...)
		return CLIB.btSoftBody_Joint_setMassmatrix(self.ptr, ...)
	end
	function META:SetErp(...)
		return CLIB.btSoftBody_Joint_setErp(self.ptr, ...)
	end
	function META:SetDrift(...)
		return CLIB.btSoftBody_Joint_setDrift(self.ptr, ...)
	end
	function META:SetDelete(...)
		return CLIB.btSoftBody_Joint_setDelete(self.ptr, ...)
	end
	function META:SetCfm(...)
		return CLIB.btSoftBody_Joint_setCfm(self.ptr, ...)
	end
	function META:Prepare(...)
		return CLIB.btSoftBody_Joint_Prepare(self.ptr, ...)
	end
	function META:GetSdrift(...)
		return CLIB.btSoftBody_Joint_getSdrift(self.ptr, ...)
	end
	function META:GetCfm(...)
		return CLIB.btSoftBody_Joint_getCfm(self.ptr, ...)
	end
	function META:GetDrift(...)
		return CLIB.btSoftBody_Joint_getDrift(self.ptr, ...)
	end
	function META:GetMassmatrix(...)
		return CLIB.btSoftBody_Joint_getMassmatrix(self.ptr, ...)
	end
end
do -- btSoftBody_Tetra
	local META = {}
	library.metatables.btSoftBody_Tetra = META
	META.__index = function(s, k) return META[k] end
	function META:SetRv(...)
		return CLIB.btSoftBody_Tetra_setRv(self.ptr, ...)
	end
	function META:GetLeaf(...)
		return CLIB.btSoftBody_Tetra_getLeaf(self.ptr, ...)
	end
	function META:GetRv(...)
		return CLIB.btSoftBody_Tetra_getRv(self.ptr, ...)
	end
	function META:GetC1(...)
		return CLIB.btSoftBody_Tetra_getC1(self.ptr, ...)
	end
	function META:SetLeaf(...)
		return CLIB.btSoftBody_Tetra_setLeaf(self.ptr, ...)
	end
	function META:SetC2(...)
		return CLIB.btSoftBody_Tetra_setC2(self.ptr, ...)
	end
	function META:SetC1(...)
		return CLIB.btSoftBody_Tetra_setC1(self.ptr, ...)
	end
	function META:GetN(...)
		return CLIB.btSoftBody_Tetra_getN(self.ptr, ...)
	end
	function META:GetC0(...)
		return CLIB.btSoftBody_Tetra_getC0(self.ptr, ...)
	end
	function META:GetC2(...)
		return CLIB.btSoftBody_Tetra_getC2(self.ptr, ...)
	end
end
do -- btPointCollector
	local META = {}
	library.metatables.btPointCollector = META
	META.__index = function(s, k) return META[k] end
	function library.CreatePointCollector(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btPointCollector_new(...)
		return self
	end
	function META:GetPointInWorld(...)
		return CLIB.btPointCollector_getPointInWorld(self.ptr, ...)
	end
	function META:SetNormalOnBInWorld(...)
		return CLIB.btPointCollector_setNormalOnBInWorld(self.ptr, ...)
	end
	function META:SetHasResult(...)
		return CLIB.btPointCollector_setHasResult(self.ptr, ...)
	end
	function META:SetDistance(...)
		return CLIB.btPointCollector_setDistance(self.ptr, ...)
	end
	function META:GetNormalOnBInWorld(...)
		return CLIB.btPointCollector_getNormalOnBInWorld(self.ptr, ...)
	end
	function META:GetHasResult(...)
		return CLIB.btPointCollector_getHasResult(self.ptr, ...)
	end
	function META:GetDistance(...)
		return CLIB.btPointCollector_getDistance(self.ptr, ...)
	end
	function META:SetPointInWorld(...)
		return CLIB.btPointCollector_setPointInWorld(self.ptr, ...)
	end
end
do -- btTriangleBuffer
	local META = {}
	library.metatables.btTriangleBuffer = META
	META.__index = function(s, k) return META[k] end
	function library.CreateTriangleBuffer(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btTriangleBuffer_new(...)
		return self
	end
	function META:GetTriangle(...)
		return CLIB.btTriangleBuffer_getTriangle(self.ptr, ...)
	end
	function META:GetNumTriangles(...)
		return CLIB.btTriangleBuffer_getNumTriangles(self.ptr, ...)
	end
	function META:ClearBuffer(...)
		return CLIB.btTriangleBuffer_clearBuffer(self.ptr, ...)
	end
end
do -- btGearConstraint
	local META = {}
	library.metatables.btGearConstraint = META
	META.__index = function(s, k) return META[k] end
	function library.CreateGearConstraint(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btGearConstraint_new(...)
		return self
	end
	function META:GetAxisA(...)
		return CLIB.btGearConstraint_getAxisA(self.ptr, ...)
	end
	function META:SetAxisA(...)
		return CLIB.btGearConstraint_setAxisA(self.ptr, ...)
	end
	function META:GetRatio(...)
		return CLIB.btGearConstraint_getRatio(self.ptr, ...)
	end
	function META:SetAxisB(...)
		return CLIB.btGearConstraint_setAxisB(self.ptr, ...)
	end
	function META:GetAxisB(...)
		return CLIB.btGearConstraint_getAxisB(self.ptr, ...)
	end
	function META:SetRatio(...)
		return CLIB.btGearConstraint_setRatio(self.ptr, ...)
	end
end
do -- btCylinderShapeX
	local META = {}
	library.metatables.btCylinderShapeX = META
	META.__index = function(s, k) return META[k] end
	function library.CreateCylinderShapeX(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btCylinderShapeX_new(...)
		return self
	end
	function library.CreateCylinderShapeX2(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btCylinderShapeX_new2(...)
		return self
	end
end
do -- btDbvtBroadphase
	local META = {}
	library.metatables.btDbvtBroadphase = META
	META.__index = function(s, k) return META[k] end
	function library.CreateDbvtBroadphase(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btDbvtBroadphase_new(...)
		return self
	end
	function META:SetStageCurrent(...)
		return CLIB.btDbvtBroadphase_setStageCurrent(self.ptr, ...)
	end
	function META:SetVelocityPrediction(...)
		return CLIB.btDbvtBroadphase_setVelocityPrediction(self.ptr, ...)
	end
	function META:SetPaircache(...)
		return CLIB.btDbvtBroadphase_setPaircache(self.ptr, ...)
	end
	function META:GetGid(...)
		return CLIB.btDbvtBroadphase_getGid(self.ptr, ...)
	end
	function META:GetStageCurrent(...)
		return CLIB.btDbvtBroadphase_getStageCurrent(self.ptr, ...)
	end
	function META:SetAabbForceUpdate(...)
		return CLIB.btDbvtBroadphase_setAabbForceUpdate(self.ptr, ...)
	end
	function META:GetSets(...)
		return CLIB.btDbvtBroadphase_getSets(self.ptr, ...)
	end
	function META:SetPid(...)
		return CLIB.btDbvtBroadphase_setPid(self.ptr, ...)
	end
	function META:Benchmark(...)
		return CLIB.btDbvtBroadphase_benchmark(self.ptr, ...)
	end
	function META:SetGid(...)
		return CLIB.btDbvtBroadphase_setGid(self.ptr, ...)
	end
	function META:GetNewpairs(...)
		return CLIB.btDbvtBroadphase_getNewpairs(self.ptr, ...)
	end
	function META:GetDeferedcollide(...)
		return CLIB.btDbvtBroadphase_getDeferedcollide(self.ptr, ...)
	end
	function META:SetReleasepaircache(...)
		return CLIB.btDbvtBroadphase_setReleasepaircache(self.ptr, ...)
	end
	function META:GetReleasepaircache(...)
		return CLIB.btDbvtBroadphase_getReleasepaircache(self.ptr, ...)
	end
	function META:SetPrediction(...)
		return CLIB.btDbvtBroadphase_setPrediction(self.ptr, ...)
	end
	function META:Collide(...)
		return CLIB.btDbvtBroadphase_collide(self.ptr, ...)
	end
	function META:GetNeedcleanup(...)
		return CLIB.btDbvtBroadphase_getNeedcleanup(self.ptr, ...)
	end
	function META:GetVelocityPrediction(...)
		return CLIB.btDbvtBroadphase_getVelocityPrediction(self.ptr, ...)
	end
	function META:SetCid(...)
		return CLIB.btDbvtBroadphase_setCid(self.ptr, ...)
	end
	function META:GetCupdates(...)
		return CLIB.btDbvtBroadphase_getCupdates(self.ptr, ...)
	end
	function META:SetFixedleft(...)
		return CLIB.btDbvtBroadphase_setFixedleft(self.ptr, ...)
	end
	function META:GetPrediction(...)
		return CLIB.btDbvtBroadphase_getPrediction(self.ptr, ...)
	end
	function META:GetCid(...)
		return CLIB.btDbvtBroadphase_getCid(self.ptr, ...)
	end
	function META:SetFupdates(...)
		return CLIB.btDbvtBroadphase_setFupdates(self.ptr, ...)
	end
	function META:SetNeedcleanup(...)
		return CLIB.btDbvtBroadphase_setNeedcleanup(self.ptr, ...)
	end
	function META:Optimize(...)
		return CLIB.btDbvtBroadphase_optimize(self.ptr, ...)
	end
	function META:PerformDeferredRemoval(...)
		return CLIB.btDbvtBroadphase_performDeferredRemoval(self.ptr, ...)
	end
	function META:GetPid(...)
		return CLIB.btDbvtBroadphase_getPid(self.ptr, ...)
	end
	function META:SetDupdates(...)
		return CLIB.btDbvtBroadphase_setDupdates(self.ptr, ...)
	end
	function META:SetDeferedcollide(...)
		return CLIB.btDbvtBroadphase_setDeferedcollide(self.ptr, ...)
	end
	function META:GetDupdates(...)
		return CLIB.btDbvtBroadphase_getDupdates(self.ptr, ...)
	end
	function META:GetFupdates(...)
		return CLIB.btDbvtBroadphase_getFupdates(self.ptr, ...)
	end
	function META:GetStageRoots(...)
		return CLIB.btDbvtBroadphase_getStageRoots(self.ptr, ...)
	end
	function META:SetNewpairs(...)
		return CLIB.btDbvtBroadphase_setNewpairs(self.ptr, ...)
	end
	function META:SetCupdates(...)
		return CLIB.btDbvtBroadphase_setCupdates(self.ptr, ...)
	end
	function META:GetPaircache(...)
		return CLIB.btDbvtBroadphase_getPaircache(self.ptr, ...)
	end
	function META:GetFixedleft(...)
		return CLIB.btDbvtBroadphase_getFixedleft(self.ptr, ...)
	end
end
do -- btAABB_increment
	local META = {}
	library.metatables.btAABB_increment = META
	META.__index = function(s, k) return META[k] end
	function META:Margin(...)
		return CLIB.btAABB_increment_margin(self.ptr, ...)
	end
end
do -- btSoftBodySolver
	local META = {}
	library.metatables.btSoftBodySolver = META
	META.__index = function(s, k) return META[k] end
	function META:Delete(...)
		return CLIB.btSoftBodySolver_delete(self.ptr, ...)
	end
	function META:GetNumberOfPositionIterations(...)
		return CLIB.btSoftBodySolver_getNumberOfPositionIterations(self.ptr, ...)
	end
	function META:SetNumberOfVelocityIterations(...)
		return CLIB.btSoftBodySolver_setNumberOfVelocityIterations(self.ptr, ...)
	end
	function META:SetNumberOfPositionIterations(...)
		return CLIB.btSoftBodySolver_setNumberOfPositionIterations(self.ptr, ...)
	end
	function META:PredictMotion(...)
		return CLIB.btSoftBodySolver_predictMotion(self.ptr, ...)
	end
	function META:GetNumberOfVelocityIterations(...)
		return CLIB.btSoftBodySolver_getNumberOfVelocityIterations(self.ptr, ...)
	end
	function META:CheckInitialized(...)
		return CLIB.btSoftBodySolver_checkInitialized(self.ptr, ...)
	end
	function META:UpdateSoftBodies(...)
		return CLIB.btSoftBodySolver_updateSoftBodies(self.ptr, ...)
	end
	function META:CopyBackToSoftBodies(...)
		return CLIB.btSoftBodySolver_copyBackToSoftBodies(self.ptr, ...)
	end
	function META:GetTimeScale(...)
		return CLIB.btSoftBodySolver_getTimeScale(self.ptr, ...)
	end
	function META:SolveConstraints(...)
		return CLIB.btSoftBodySolver_solveConstraints(self.ptr, ...)
	end
end
do -- btEmptyAlgorithm
	local META = {}
	library.metatables.btEmptyAlgorithm = META
	META.__index = function(s, k) return META[k] end
	function library.CreateEmptyAlgorithm(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btEmptyAlgorithm_new(...)
		return self
	end
end
do -- btCollisionShape
	local META = {}
	library.metatables.btCollisionShape = META
	META.__index = function(s, k) return META[k] end
	function META:IsInfinite(...)
		return CLIB.btCollisionShape_isInfinite(self.ptr, ...)
	end
	function META:GetUserPointer(...)
		return CLIB.btCollisionShape_getUserPointer(self.ptr, ...)
	end
	function META:IsConvex2d(...)
		return CLIB.btCollisionShape_isConvex2d(self.ptr, ...)
	end
	function META:SerializeSingleShape(...)
		return CLIB.btCollisionShape_serializeSingleShape(self.ptr, ...)
	end
	function META:Serialize(...)
		return CLIB.btCollisionShape_serialize(self.ptr, ...)
	end
	function META:GetAngularMotionDisc(...)
		return CLIB.btCollisionShape_getAngularMotionDisc(self.ptr, ...)
	end
	function META:IsConvex(...)
		return CLIB.btCollisionShape_isConvex(self.ptr, ...)
	end
	function META:GetAabb(...)
		return CLIB.btCollisionShape_getAabb(self.ptr, ...)
	end
	function META:CalculateTemporalAabb(...)
		return CLIB.btCollisionShape_calculateTemporalAabb(self.ptr, ...)
	end
	function META:IsConcave(...)
		return CLIB.btCollisionShape_isConcave(self.ptr, ...)
	end
	function META:IsCompound(...)
		return CLIB.btCollisionShape_isCompound(self.ptr, ...)
	end
	function META:IsSoftBody(...)
		return CLIB.btCollisionShape_isSoftBody(self.ptr, ...)
	end
	function META:Delete(...)
		return CLIB.btCollisionShape_delete(self.ptr, ...)
	end
	function META:GetLocalScaling(...)
		return CLIB.btCollisionShape_getLocalScaling(self.ptr, ...)
	end
	function META:IsNonMoving(...)
		return CLIB.btCollisionShape_isNonMoving(self.ptr, ...)
	end
	function META:GetAnisotropicRollingFrictionDirection(...)
		return CLIB.btCollisionShape_getAnisotropicRollingFrictionDirection(self.ptr, ...)
	end
	function META:GetUserIndex(...)
		return CLIB.btCollisionShape_getUserIndex(self.ptr, ...)
	end
	function META:GetName(...)
		return CLIB.btCollisionShape_getName(self.ptr, ...)
	end
	function META:CalculateLocalInertia(...)
		return CLIB.btCollisionShape_calculateLocalInertia(self.ptr, ...)
	end
	function META:GetBoundingSphere(...)
		return CLIB.btCollisionShape_getBoundingSphere(self.ptr, ...)
	end
	function META:GetShapeType(...)
		return CLIB.btCollisionShape_getShapeType(self.ptr, ...)
	end
	function META:GetMargin(...)
		return CLIB.btCollisionShape_getMargin(self.ptr, ...)
	end
	function META:GetContactBreakingThreshold(...)
		return CLIB.btCollisionShape_getContactBreakingThreshold(self.ptr, ...)
	end
	function META:IsPolyhedral(...)
		return CLIB.btCollisionShape_isPolyhedral(self.ptr, ...)
	end
	function META:SetMargin(...)
		return CLIB.btCollisionShape_setMargin(self.ptr, ...)
	end
	function META:SetLocalScaling(...)
		return CLIB.btCollisionShape_setLocalScaling(self.ptr, ...)
	end
	function META:SetUserPointer(...)
		return CLIB.btCollisionShape_setUserPointer(self.ptr, ...)
	end
	function META:SetUserIndex(...)
		return CLIB.btCollisionShape_setUserIndex(self.ptr, ...)
	end
	function META:CalculateSerializeBufferSize(...)
		return CLIB.btCollisionShape_calculateSerializeBufferSize(self.ptr, ...)
	end
end
do -- btWorldImporter
	local META = {}
	library.metatables.btWorldImporter = META
	META.__index = function(s, k) return META[k] end
	function library.CreateWorldImporter(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btWorldImporter_new(...)
		return self
	end
	function META:CreateGearConstraint(...)
		return CLIB.btWorldImporter_createGearConstraint(self.ptr, ...)
	end
	function META:CreateCapsuleShapeY(...)
		return CLIB.btWorldImporter_createCapsuleShapeY(self.ptr, ...)
	end
	function META:CreateConeShapeY(...)
		return CLIB.btWorldImporter_createConeShapeY(self.ptr, ...)
	end
	function META:CreateSliderConstraint2(...)
		return CLIB.btWorldImporter_createSliderConstraint2(self.ptr, ...)
	end
	function META:CreateMultiSphereShape(...)
		return CLIB.btWorldImporter_createMultiSphereShape(self.ptr, ...)
	end
	function META:Delete(...)
		return CLIB.btWorldImporter_delete(self.ptr, ...)
	end
	function META:CreateCapsuleShapeX(...)
		return CLIB.btWorldImporter_createCapsuleShapeX(self.ptr, ...)
	end
	function META:GetVerboseMode(...)
		return CLIB.btWorldImporter_getVerboseMode(self.ptr, ...)
	end
	function META:GetRigidBodyByIndex(...)
		return CLIB.btWorldImporter_getRigidBodyByIndex(self.ptr, ...)
	end
	function META:GetNumTriangleInfoMaps(...)
		return CLIB.btWorldImporter_getNumTriangleInfoMaps(self.ptr, ...)
	end
	function META:GetNumBvhs(...)
		return CLIB.btWorldImporter_getNumBvhs(self.ptr, ...)
	end
	function META:GetConstraintByName(...)
		return CLIB.btWorldImporter_getConstraintByName(self.ptr, ...)
	end
	function META:GetCollisionShapeByName(...)
		return CLIB.btWorldImporter_getCollisionShapeByName(self.ptr, ...)
	end
	function META:CreateTriangleMeshContainer(...)
		return CLIB.btWorldImporter_createTriangleMeshContainer(self.ptr, ...)
	end
	function META:CreateStridingMeshInterfaceData(...)
		return CLIB.btWorldImporter_createStridingMeshInterfaceData(self.ptr, ...)
	end
	function META:CreateSphereShape(...)
		return CLIB.btWorldImporter_createSphereShape(self.ptr, ...)
	end
	function META:CreateSliderConstraint(...)
		return CLIB.btWorldImporter_createSliderConstraint(self.ptr, ...)
	end
	function META:CreatePlaneShape(...)
		return CLIB.btWorldImporter_createPlaneShape(self.ptr, ...)
	end
	function META:CreateOptimizedBvh(...)
		return CLIB.btWorldImporter_createOptimizedBvh(self.ptr, ...)
	end
	function META:CreateMeshInterface(...)
		return CLIB.btWorldImporter_createMeshInterface(self.ptr, ...)
	end
	function META:CreateHingeConstraint4(...)
		return CLIB.btWorldImporter_createHingeConstraint4(self.ptr, ...)
	end
	function META:CreateHingeConstraint3(...)
		return CLIB.btWorldImporter_createHingeConstraint3(self.ptr, ...)
	end
	function META:CreateHingeConstraint2(...)
		return CLIB.btWorldImporter_createHingeConstraint2(self.ptr, ...)
	end
	function META:CreateHingeConstraint(...)
		return CLIB.btWorldImporter_createHingeConstraint(self.ptr, ...)
	end
	function META:CreateGeneric6DofSpring2Constraint(...)
		return CLIB.btWorldImporter_createGeneric6DofSpring2Constraint(self.ptr, ...)
	end
	function META:CreateGeneric6DofConstraint(...)
		return CLIB.btWorldImporter_createGeneric6DofConstraint(self.ptr, ...)
	end
	function META:CreateCylinderShapeY(...)
		return CLIB.btWorldImporter_createCylinderShapeY(self.ptr, ...)
	end
	function META:CreateCylinderShapeX(...)
		return CLIB.btWorldImporter_createCylinderShapeX(self.ptr, ...)
	end
	function META:CreateConvexTriangleMeshShape(...)
		return CLIB.btWorldImporter_createConvexTriangleMeshShape(self.ptr, ...)
	end
	function META:CreateConeTwistConstraint2(...)
		return CLIB.btWorldImporter_createConeTwistConstraint2(self.ptr, ...)
	end
	function META:CreateCompoundShape(...)
		return CLIB.btWorldImporter_createCompoundShape(self.ptr, ...)
	end
	function META:CreateBvhTriangleMeshShape(...)
		return CLIB.btWorldImporter_createBvhTriangleMeshShape(self.ptr, ...)
	end
	function META:CreateBoxShape(...)
		return CLIB.btWorldImporter_createBoxShape(self.ptr, ...)
	end
	function META:CreateGeneric6DofSpringConstraint(...)
		return CLIB.btWorldImporter_createGeneric6DofSpringConstraint(self.ptr, ...)
	end
	function META:GetNumRigidBodies(...)
		return CLIB.btWorldImporter_getNumRigidBodies(self.ptr, ...)
	end
	function META:CreateScaledTrangleMeshShape(...)
		return CLIB.btWorldImporter_createScaledTrangleMeshShape(self.ptr, ...)
	end
	function META:CreateConvexHullShape(...)
		return CLIB.btWorldImporter_createConvexHullShape(self.ptr, ...)
	end
	function META:GetNumConstraints(...)
		return CLIB.btWorldImporter_getNumConstraints(self.ptr, ...)
	end
	function META:GetNumCollisionShapes(...)
		return CLIB.btWorldImporter_getNumCollisionShapes(self.ptr, ...)
	end
	function META:GetConstraintByIndex(...)
		return CLIB.btWorldImporter_getConstraintByIndex(self.ptr, ...)
	end
	function META:CreateCylinderShapeZ(...)
		return CLIB.btWorldImporter_createCylinderShapeZ(self.ptr, ...)
	end
	function META:GetBvhByIndex(...)
		return CLIB.btWorldImporter_getBvhByIndex(self.ptr, ...)
	end
	function META:CreateCollisionObject(...)
		return CLIB.btWorldImporter_createCollisionObject(self.ptr, ...)
	end
	function META:CreatePoint2PointConstraint2(...)
		return CLIB.btWorldImporter_createPoint2PointConstraint2(self.ptr, ...)
	end
	function META:CreateRigidBody(...)
		return CLIB.btWorldImporter_createRigidBody(self.ptr, ...)
	end
	function META:GetRigidBodyByName(...)
		return CLIB.btWorldImporter_getRigidBodyByName(self.ptr, ...)
	end
	function META:DeleteAllData(...)
		return CLIB.btWorldImporter_deleteAllData(self.ptr, ...)
	end
	function META:GetNameForPointer(...)
		return CLIB.btWorldImporter_getNameForPointer(self.ptr, ...)
	end
	function META:GetCollisionShapeByIndex(...)
		return CLIB.btWorldImporter_getCollisionShapeByIndex(self.ptr, ...)
	end
	function META:CreateGimpactShape(...)
		return CLIB.btWorldImporter_createGimpactShape(self.ptr, ...)
	end
	function META:SetDynamicsWorldInfo(...)
		return CLIB.btWorldImporter_setDynamicsWorldInfo(self.ptr, ...)
	end
	function META:GetTriangleInfoMapByIndex(...)
		return CLIB.btWorldImporter_getTriangleInfoMapByIndex(self.ptr, ...)
	end
	function META:CreateConeShapeZ(...)
		return CLIB.btWorldImporter_createConeShapeZ(self.ptr, ...)
	end
	function META:CreateTriangleInfoMap(...)
		return CLIB.btWorldImporter_createTriangleInfoMap(self.ptr, ...)
	end
	function META:CreatePoint2PointConstraint(...)
		return CLIB.btWorldImporter_createPoint2PointConstraint(self.ptr, ...)
	end
	function META:CreateConeTwistConstraint(...)
		return CLIB.btWorldImporter_createConeTwistConstraint(self.ptr, ...)
	end
	function META:CreateConeShapeX(...)
		return CLIB.btWorldImporter_createConeShapeX(self.ptr, ...)
	end
	function META:CreateGeneric6DofConstraint2(...)
		return CLIB.btWorldImporter_createGeneric6DofConstraint2(self.ptr, ...)
	end
	function META:SetVerboseMode(...)
		return CLIB.btWorldImporter_setVerboseMode(self.ptr, ...)
	end
	function META:CreateCapsuleShapeZ(...)
		return CLIB.btWorldImporter_createCapsuleShapeZ(self.ptr, ...)
	end
end
do -- btNullPairCache
	local META = {}
	library.metatables.btNullPairCache = META
	META.__index = function(s, k) return META[k] end
	function library.CreateNullPairCache(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btNullPairCache_new(...)
		return self
	end
end
do -- btCompoundShape
	local META = {}
	library.metatables.btCompoundShape = META
	META.__index = function(s, k) return META[k] end
	function library.CreateCompoundShape(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btCompoundShape_new(...)
		return self
	end
	function META:GetChildTransform(...)
		return CLIB.btCompoundShape_getChildTransform(self.ptr, ...)
	end
	function META:CalculatePrincipalAxisTransform(...)
		return CLIB.btCompoundShape_calculatePrincipalAxisTransform(self.ptr, ...)
	end
	function META:GetDynamicAabbTree(...)
		return CLIB.btCompoundShape_getDynamicAabbTree(self.ptr, ...)
	end
	function META:UpdateChildTransform(...)
		return CLIB.btCompoundShape_updateChildTransform(self.ptr, ...)
	end
	function META:GetChildList(...)
		return CLIB.btCompoundShape_getChildList(self.ptr, ...)
	end
	function META:GetNumChildShapes(...)
		return CLIB.btCompoundShape_getNumChildShapes(self.ptr, ...)
	end
	function META:RemoveChildShapeByIndex(...)
		return CLIB.btCompoundShape_removeChildShapeByIndex(self.ptr, ...)
	end
	function META:GetUpdateRevision(...)
		return CLIB.btCompoundShape_getUpdateRevision(self.ptr, ...)
	end
	function META:RemoveChildShape(...)
		return CLIB.btCompoundShape_removeChildShape(self.ptr, ...)
	end
	function META:RecalculateLocalAabb(...)
		return CLIB.btCompoundShape_recalculateLocalAabb(self.ptr, ...)
	end
	function META:AddChildShape(...)
		return CLIB.btCompoundShape_addChildShape(self.ptr, ...)
	end
	function META:CreateAabbTreeFromChildren(...)
		return CLIB.btCompoundShape_createAabbTreeFromChildren(self.ptr, ...)
	end
	function META:GetChildShape(...)
		return CLIB.btCompoundShape_getChildShape(self.ptr, ...)
	end
end
do -- btDynamicsWorld
	local META = {}
	library.metatables.btDynamicsWorld = META
	function META:__index(k)
		local v

		v = META[k]
		if v ~= nil then
			return v
		end
		v = library.metatables.btCollisionWorld.__index(self, k)
		if v ~= nil then
			return v
		end
	end
	function META:AddConstraint(...)
		return CLIB.btDynamicsWorld_addConstraint(self.ptr, ...)
	end
	function META:AddAction(...)
		return CLIB.btDynamicsWorld_addAction(self.ptr, ...)
	end
	function META:SetWorldUserInfo(...)
		return CLIB.btDynamicsWorld_setWorldUserInfo(self.ptr, ...)
	end
	function META:RemoveAction(...)
		return CLIB.btDynamicsWorld_removeAction(self.ptr, ...)
	end
	function META:GetWorldUserInfo(...)
		return CLIB.btDynamicsWorld_getWorldUserInfo(self.ptr, ...)
	end
	function META:GetSolverInfo(...)
		return CLIB.btDynamicsWorld_getSolverInfo(self.ptr, ...)
	end
	function META:SynchronizeMotionStates(...)
		return CLIB.btDynamicsWorld_synchronizeMotionStates(self.ptr, ...)
	end
	function META:SetConstraintSolver(...)
		return CLIB.btDynamicsWorld_setConstraintSolver(self.ptr, ...)
	end
	function META:GetWorldType(...)
		return CLIB.btDynamicsWorld_getWorldType(self.ptr, ...)
	end
	function META:AddRigidBody2(...)
		return CLIB.btDynamicsWorld_addRigidBody2(self.ptr, ...)
	end
	function META:GetGravity(...)
		return CLIB.btDynamicsWorld_getGravity(self.ptr, ...)
	end
	function META:StepSimulation(...)
		return CLIB.btDynamicsWorld_stepSimulation(self.ptr, ...)
	end
	function META:ClearForces(...)
		return CLIB.btDynamicsWorld_clearForces(self.ptr, ...)
	end
	function META:AddRigidBody(...)
		return CLIB.btDynamicsWorld_addRigidBody(self.ptr, ...)
	end
	function META:GetConstraint(...)
		return CLIB.btDynamicsWorld_getConstraint(self.ptr, ...)
	end
	function META:GetConstraintSolver(...)
		return CLIB.btDynamicsWorld_getConstraintSolver(self.ptr, ...)
	end
	function META:GetNumConstraints(...)
		return CLIB.btDynamicsWorld_getNumConstraints(self.ptr, ...)
	end
	function META:RemoveRigidBody(...)
		return CLIB.btDynamicsWorld_removeRigidBody(self.ptr, ...)
	end
	function META:RemoveConstraint(...)
		return CLIB.btDynamicsWorld_removeConstraint(self.ptr, ...)
	end
	function META:SetGravity(...)
		return CLIB.btDynamicsWorld_setGravity(self.ptr, ...)
	end
	function META:SetInternalTickCallback(...)
		return CLIB.btDynamicsWorld_setInternalTickCallback(self.ptr, ...)
	end
end
do -- btMultibodyLink
	local META = {}
	library.metatables.btMultibodyLink = META
	META.__index = function(s, k) return META[k] end
	function META:GetCfgOffset(...)
		return CLIB.btMultibodyLink_getCfgOffset(self.ptr, ...)
	end
	function META:GetCachedWorldTransform(...)
		return CLIB.btMultibodyLink_getCachedWorldTransform(self.ptr, ...)
	end
	function META:GetJointType(...)
		return CLIB.btMultibodyLink_getJointType(self.ptr, ...)
	end
	function META:GetAppliedConstraintTorque(...)
		return CLIB.btMultibodyLink_getAppliedConstraintTorque(self.ptr, ...)
	end
	function META:GetEVector(...)
		return CLIB.btMultibodyLink_getEVector(self.ptr, ...)
	end
	function META:GetDofCount(...)
		return CLIB.btMultibodyLink_getDofCount(self.ptr, ...)
	end
	function META:GetParent(...)
		return CLIB.btMultibodyLink_getParent(self.ptr, ...)
	end
	function META:GetFlags(...)
		return CLIB.btMultibodyLink_getFlags(self.ptr, ...)
	end
	function META:GetAxisTop(...)
		return CLIB.btMultibodyLink_getAxisTop(self.ptr, ...)
	end
	function META:SetJointDamping(...)
		return CLIB.btMultibodyLink_setJointDamping(self.ptr, ...)
	end
	function META:GetUserPtr(...)
		return CLIB.btMultibodyLink_getUserPtr(self.ptr, ...)
	end
	function META:GetCachedRotParentToThis(...)
		return CLIB.btMultibodyLink_getCachedRotParentToThis(self.ptr, ...)
	end
	function META:SetAppliedTorque(...)
		return CLIB.btMultibodyLink_setAppliedTorque(self.ptr, ...)
	end
	function META:SetAppliedConstraintTorque(...)
		return CLIB.btMultibodyLink_setAppliedConstraintTorque(self.ptr, ...)
	end
	function META:SetEVector(...)
		return CLIB.btMultibodyLink_setEVector(self.ptr, ...)
	end
	function META:SetJointFeedback(...)
		return CLIB.btMultibodyLink_setJointFeedback(self.ptr, ...)
	end
	function META:GetAppliedTorque(...)
		return CLIB.btMultibodyLink_getAppliedTorque(self.ptr, ...)
	end
	function META:GetInertiaLocal(...)
		return CLIB.btMultibodyLink_getInertiaLocal(self.ptr, ...)
	end
	function META:SetAxisBottom2(...)
		return CLIB.btMultibodyLink_setAxisBottom2(self.ptr, ...)
	end
	function META:GetPosVarCount(...)
		return CLIB.btMultibodyLink_getPosVarCount(self.ptr, ...)
	end
	function META:SetCfgOffset(...)
		return CLIB.btMultibodyLink_setCfgOffset(self.ptr, ...)
	end
	function META:GetAppliedForce(...)
		return CLIB.btMultibodyLink_getAppliedForce(self.ptr, ...)
	end
	function META:SetPosVarCount(...)
		return CLIB.btMultibodyLink_setPosVarCount(self.ptr, ...)
	end
	function META:GetJointName(...)
		return CLIB.btMultibodyLink_getJointName(self.ptr, ...)
	end
	function META:GetAppliedConstraintForce(...)
		return CLIB.btMultibodyLink_getAppliedConstraintForce(self.ptr, ...)
	end
	function META:SetLinkName(...)
		return CLIB.btMultibodyLink_setLinkName(self.ptr, ...)
	end
	function META:GetAxes(...)
		return CLIB.btMultibodyLink_getAxes(self.ptr, ...)
	end
	function META:SetAxisTop(...)
		return CLIB.btMultibodyLink_setAxisTop(self.ptr, ...)
	end
	function META:GetDofOffset(...)
		return CLIB.btMultibodyLink_getDofOffset(self.ptr, ...)
	end
	function META:GetAxisBottom(...)
		return CLIB.btMultibodyLink_getAxisBottom(self.ptr, ...)
	end
	function META:GetJointPos(...)
		return CLIB.btMultibodyLink_getJointPos(self.ptr, ...)
	end
	function META:GetJointTorque(...)
		return CLIB.btMultibodyLink_getJointTorque(self.ptr, ...)
	end
	function META:UpdateCacheMultiDof(...)
		return CLIB.btMultibodyLink_updateCacheMultiDof(self.ptr, ...)
	end
	function META:SetUserPtr(...)
		return CLIB.btMultibodyLink_setUserPtr(self.ptr, ...)
	end
	function META:SetZeroRotParentToThis(...)
		return CLIB.btMultibodyLink_setZeroRotParentToThis(self.ptr, ...)
	end
	function META:SetParent(...)
		return CLIB.btMultibodyLink_setParent(self.ptr, ...)
	end
	function META:SetMass(...)
		return CLIB.btMultibodyLink_setMass(self.ptr, ...)
	end
	function META:SetJointType(...)
		return CLIB.btMultibodyLink_setJointType(self.ptr, ...)
	end
	function META:SetJointName(...)
		return CLIB.btMultibodyLink_setJointName(self.ptr, ...)
	end
	function META:SetJointFriction(...)
		return CLIB.btMultibodyLink_setJointFriction(self.ptr, ...)
	end
	function META:SetFlags(...)
		return CLIB.btMultibodyLink_setFlags(self.ptr, ...)
	end
	function META:SetDVector(...)
		return CLIB.btMultibodyLink_setDVector(self.ptr, ...)
	end
	function META:SetDofOffset(...)
		return CLIB.btMultibodyLink_setDofOffset(self.ptr, ...)
	end
	function META:SetDofCount(...)
		return CLIB.btMultibodyLink_setDofCount(self.ptr, ...)
	end
	function META:SetCollider(...)
		return CLIB.btMultibodyLink_setCollider(self.ptr, ...)
	end
	function META:SetCachedWorldTransform(...)
		return CLIB.btMultibodyLink_setCachedWorldTransform(self.ptr, ...)
	end
	function META:SetCachedRVector(...)
		return CLIB.btMultibodyLink_setCachedRVector(self.ptr, ...)
	end
	function META:SetCachedRotParentToThis(...)
		return CLIB.btMultibodyLink_setCachedRotParentToThis(self.ptr, ...)
	end
	function META:SetAxisTop2(...)
		return CLIB.btMultibodyLink_setAxisTop2(self.ptr, ...)
	end
	function META:SetAxisBottom(...)
		return CLIB.btMultibodyLink_setAxisBottom(self.ptr, ...)
	end
	function META:SetAppliedForce(...)
		return CLIB.btMultibodyLink_setAppliedForce(self.ptr, ...)
	end
	function META:SetAppliedConstraintForce(...)
		return CLIB.btMultibodyLink_setAppliedConstraintForce(self.ptr, ...)
	end
	function META:GetDVector(...)
		return CLIB.btMultibodyLink_getDVector(self.ptr, ...)
	end
	function META:GetLinkName(...)
		return CLIB.btMultibodyLink_getLinkName(self.ptr, ...)
	end
	function META:GetCachedRVector(...)
		return CLIB.btMultibodyLink_getCachedRVector(self.ptr, ...)
	end
	function META:GetJointFriction(...)
		return CLIB.btMultibodyLink_getJointFriction(self.ptr, ...)
	end
	function META:GetZeroRotParentToThis(...)
		return CLIB.btMultibodyLink_getZeroRotParentToThis(self.ptr, ...)
	end
	function META:GetAbsFrameTotVelocity(...)
		return CLIB.btMultibodyLink_getAbsFrameTotVelocity(self.ptr, ...)
	end
	function META:GetCollider(...)
		return CLIB.btMultibodyLink_getCollider(self.ptr, ...)
	end
	function META:SetInertiaLocal(...)
		return CLIB.btMultibodyLink_setInertiaLocal(self.ptr, ...)
	end
	function META:GetAbsFrameLocVelocity(...)
		return CLIB.btMultibodyLink_getAbsFrameLocVelocity(self.ptr, ...)
	end
	function META:GetJointDamping(...)
		return CLIB.btMultibodyLink_getJointDamping(self.ptr, ...)
	end
	function META:GetJointFeedback(...)
		return CLIB.btMultibodyLink_getJointFeedback(self.ptr, ...)
	end
	function META:GetMass(...)
		return CLIB.btMultibodyLink_getMass(self.ptr, ...)
	end
end
do -- btTransformUtil
	local META = {}
	library.metatables.btTransformUtil = META
	META.__index = function(s, k) return META[k] end
	function META:CalculateVelocityQuaternion(...)
		return CLIB.btTransformUtil_calculateVelocityQuaternion(self.ptr, ...)
	end
	function META:CalculateDiffAxisAngleQuaternion(...)
		return CLIB.btTransformUtil_calculateDiffAxisAngleQuaternion(self.ptr, ...)
	end
	function META:IntegrateTransform(...)
		return CLIB.btTransformUtil_integrateTransform(self.ptr, ...)
	end
	function META:CalculateDiffAxisAngle(...)
		return CLIB.btTransformUtil_calculateDiffAxisAngle(self.ptr, ...)
	end
	function META:CalculateVelocity(...)
		return CLIB.btTransformUtil_calculateVelocity(self.ptr, ...)
	end
end
do -- btSoftBody_Note
	local META = {}
	library.metatables.btSoftBody_Note = META
	META.__index = function(s, k) return META[k] end
	function META:GetCoords(...)
		return CLIB.btSoftBody_Note_getCoords(self.ptr, ...)
	end
	function META:GetText(...)
		return CLIB.btSoftBody_Note_getText(self.ptr, ...)
	end
	function META:SetText(...)
		return CLIB.btSoftBody_Note_setText(self.ptr, ...)
	end
	function META:SetRank(...)
		return CLIB.btSoftBody_Note_setRank(self.ptr, ...)
	end
	function META:SetOffset(...)
		return CLIB.btSoftBody_Note_setOffset(self.ptr, ...)
	end
	function META:GetOffset(...)
		return CLIB.btSoftBody_Note_getOffset(self.ptr, ...)
	end
	function META:GetNodes(...)
		return CLIB.btSoftBody_Note_getNodes(self.ptr, ...)
	end
	function META:GetRank(...)
		return CLIB.btSoftBody_Note_getRank(self.ptr, ...)
	end
end
do -- btCapsuleShapeZ
	local META = {}
	library.metatables.btCapsuleShapeZ = META
	META.__index = function(s, k) return META[k] end
	function library.CreateCapsuleShapeZ(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btCapsuleShapeZ_new(...)
		return self
	end
end
do -- btSoftBody_Link
	local META = {}
	library.metatables.btSoftBody_Link = META
	META.__index = function(s, k) return META[k] end
	function library.CreateSoftBody_Link(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btSoftBody_Link_new2(...)
		return self
	end
	function library.CreateSoftBody_Link2(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btSoftBody_Link_new(...)
		return self
	end
	function META:GetC2(...)
		return CLIB.btSoftBody_Link_getC2(self.ptr, ...)
	end
	function META:GetRl(...)
		return CLIB.btSoftBody_Link_getRl(self.ptr, ...)
	end
	function META:SetC0(...)
		return CLIB.btSoftBody_Link_setC0(self.ptr, ...)
	end
	function META:SetRl(...)
		return CLIB.btSoftBody_Link_setRl(self.ptr, ...)
	end
	function META:SetC3(...)
		return CLIB.btSoftBody_Link_setC3(self.ptr, ...)
	end
	function META:SetC2(...)
		return CLIB.btSoftBody_Link_setC2(self.ptr, ...)
	end
	function META:SetC1(...)
		return CLIB.btSoftBody_Link_setC1(self.ptr, ...)
	end
	function META:SetBbending(...)
		return CLIB.btSoftBody_Link_setBbending(self.ptr, ...)
	end
	function META:GetN(...)
		return CLIB.btSoftBody_Link_getN(self.ptr, ...)
	end
	function META:GetC1(...)
		return CLIB.btSoftBody_Link_getC1(self.ptr, ...)
	end
	function META:GetC0(...)
		return CLIB.btSoftBody_Link_getC0(self.ptr, ...)
	end
	function META:GetC3(...)
		return CLIB.btSoftBody_Link_getC3(self.ptr, ...)
	end
	function META:GetBbending(...)
		return CLIB.btSoftBody_Link_getBbending(self.ptr, ...)
	end
	function META:Delete(...)
		return CLIB.btSoftBody_Link_delete(self.ptr, ...)
	end
end
do -- btCapsuleShapeX
	local META = {}
	library.metatables.btCapsuleShapeX = META
	META.__index = function(s, k) return META[k] end
	function library.CreateCapsuleShapeX(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btCapsuleShapeX_new(...)
		return self
	end
end
do -- btJointFeedback
	local META = {}
	library.metatables.btJointFeedback = META
	META.__index = function(s, k) return META[k] end
	function library.CreateJointFeedback(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btJointFeedback_new(...)
		return self
	end
	function META:GetAppliedTorqueBodyB(...)
		return CLIB.btJointFeedback_getAppliedTorqueBodyB(self.ptr, ...)
	end
	function META:SetAppliedForceBodyA(...)
		return CLIB.btJointFeedback_setAppliedForceBodyA(self.ptr, ...)
	end
	function META:SetAppliedTorqueBodyA(...)
		return CLIB.btJointFeedback_setAppliedTorqueBodyA(self.ptr, ...)
	end
	function META:GetAppliedForceBodyA(...)
		return CLIB.btJointFeedback_getAppliedForceBodyA(self.ptr, ...)
	end
	function META:SetAppliedForceBodyB(...)
		return CLIB.btJointFeedback_setAppliedForceBodyB(self.ptr, ...)
	end
	function META:GetAppliedTorqueBodyA(...)
		return CLIB.btJointFeedback_getAppliedTorqueBodyA(self.ptr, ...)
	end
	function META:SetAppliedTorqueBodyB(...)
		return CLIB.btJointFeedback_setAppliedTorqueBodyB(self.ptr, ...)
	end
	function META:GetAppliedForceBodyB(...)
		return CLIB.btJointFeedback_getAppliedForceBodyB(self.ptr, ...)
	end
	function META:Delete(...)
		return CLIB.btJointFeedback_delete(self.ptr, ...)
	end
end
do -- btBvhTree_build
	local META = {}
	library.metatables.btBvhTree_build = META
	META.__index = function(s, k) return META[k] end
	function META:Tree(...)
		return CLIB.btBvhTree_build_tree(self.ptr, ...)
	end
end
do -- btStorageResult
	local META = {}
	library.metatables.btStorageResult = META
	META.__index = function(s, k) return META[k] end
	function META:SetNormalOnSurfaceB(...)
		return CLIB.btStorageResult_setNormalOnSurfaceB(self.ptr, ...)
	end
	function META:GetNormalOnSurfaceB(...)
		return CLIB.btStorageResult_getNormalOnSurfaceB(self.ptr, ...)
	end
	function META:GetDistance(...)
		return CLIB.btStorageResult_getDistance(self.ptr, ...)
	end
	function META:SetDistance(...)
		return CLIB.btStorageResult_setDistance(self.ptr, ...)
	end
	function META:SetClosestPointInB(...)
		return CLIB.btStorageResult_setClosestPointInB(self.ptr, ...)
	end
	function META:GetClosestPointInB(...)
		return CLIB.btStorageResult_getClosestPointInB(self.ptr, ...)
	end
end
do -- btSoftBody_Body
	local META = {}
	library.metatables.btSoftBody_Body = META
	META.__index = function(s, k) return META[k] end
	function library.CreateSoftBody_Body(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btSoftBody_Body_new3(...)
		return self
	end
	function library.CreateSoftBody_Body2(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btSoftBody_Body_new2(...)
		return self
	end
	function library.CreateSoftBody_Body3(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btSoftBody_Body_new(...)
		return self
	end
	function META:ApplyAImpulse(...)
		return CLIB.btSoftBody_Body_applyAImpulse(self.ptr, ...)
	end
	function META:LinearVelocity(...)
		return CLIB.btSoftBody_Body_linearVelocity(self.ptr, ...)
	end
	function META:AngularVelocity2(...)
		return CLIB.btSoftBody_Body_angularVelocity2(self.ptr, ...)
	end
	function META:GetSoft(...)
		return CLIB.btSoftBody_Body_getSoft(self.ptr, ...)
	end
	function META:GetRigid(...)
		return CLIB.btSoftBody_Body_getRigid(self.ptr, ...)
	end
	function META:Activate(...)
		return CLIB.btSoftBody_Body_activate(self.ptr, ...)
	end
	function META:InvWorldInertia(...)
		return CLIB.btSoftBody_Body_invWorldInertia(self.ptr, ...)
	end
	function META:ApplyVAImpulse(...)
		return CLIB.btSoftBody_Body_applyVAImpulse(self.ptr, ...)
	end
	function META:Delete(...)
		return CLIB.btSoftBody_Body_delete(self.ptr, ...)
	end
	function META:Xform(...)
		return CLIB.btSoftBody_Body_xform(self.ptr, ...)
	end
	function META:Velocity(...)
		return CLIB.btSoftBody_Body_velocity(self.ptr, ...)
	end
	function META:SetSoft(...)
		return CLIB.btSoftBody_Body_setSoft(self.ptr, ...)
	end
	function META:SetRigid(...)
		return CLIB.btSoftBody_Body_setRigid(self.ptr, ...)
	end
	function META:SetCollisionObject(...)
		return CLIB.btSoftBody_Body_setCollisionObject(self.ptr, ...)
	end
	function META:InvMass(...)
		return CLIB.btSoftBody_Body_invMass(self.ptr, ...)
	end
	function META:GetCollisionObject(...)
		return CLIB.btSoftBody_Body_getCollisionObject(self.ptr, ...)
	end
	function META:ApplyVImpulse(...)
		return CLIB.btSoftBody_Body_applyVImpulse(self.ptr, ...)
	end
	function META:ApplyImpulse(...)
		return CLIB.btSoftBody_Body_applyImpulse(self.ptr, ...)
	end
	function META:ApplyDImpulse(...)
		return CLIB.btSoftBody_Body_applyDImpulse(self.ptr, ...)
	end
	function META:ApplyDCImpulse(...)
		return CLIB.btSoftBody_Body_applyDCImpulse(self.ptr, ...)
	end
	function META:ApplyDAImpulse(...)
		return CLIB.btSoftBody_Body_applyDAImpulse(self.ptr, ...)
	end
	function META:AngularVelocity(...)
		return CLIB.btSoftBody_Body_angularVelocity(self.ptr, ...)
	end
end
do -- btTriangleShape
	local META = {}
	library.metatables.btTriangleShape = META
	META.__index = function(s, k) return META[k] end
	function library.CreateTriangleShape(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btTriangleShape_new2(...)
		return self
	end
	function library.CreateTriangleShape2(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btTriangleShape_new(...)
		return self
	end
	function META:GetVertices1(...)
		return CLIB.btTriangleShape_getVertices1(self.ptr, ...)
	end
	function META:GetVertexPtr(...)
		return CLIB.btTriangleShape_getVertexPtr(self.ptr, ...)
	end
	function META:GetPlaneEquation(...)
		return CLIB.btTriangleShape_getPlaneEquation(self.ptr, ...)
	end
	function META:CalcNormal(...)
		return CLIB.btTriangleShape_calcNormal(self.ptr, ...)
	end
end
do -- btCylinderShape
	local META = {}
	library.metatables.btCylinderShape = META
	META.__index = function(s, k) return META[k] end
	function library.CreateCylinderShape(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btCylinderShape_new(...)
		return self
	end
	function library.CreateCylinderShape2(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btCylinderShape_new2(...)
		return self
	end
	function META:GetRadius(...)
		return CLIB.btCylinderShape_getRadius(self.ptr, ...)
	end
	function META:GetHalfExtentsWithoutMargin(...)
		return CLIB.btCylinderShape_getHalfExtentsWithoutMargin(self.ptr, ...)
	end
	function META:GetHalfExtentsWithMargin(...)
		return CLIB.btCylinderShape_getHalfExtentsWithMargin(self.ptr, ...)
	end
	function META:GetUpAxis(...)
		return CLIB.btCylinderShape_getUpAxis(self.ptr, ...)
	end
end
do -- btUsageBitfield
	local META = {}
	library.metatables.btUsageBitfield = META
	META.__index = function(s, k) return META[k] end
	function META:GetUnused3(...)
		return CLIB.btUsageBitfield_getUnused3(self.ptr, ...)
	end
	function META:SetUsedVertexD(...)
		return CLIB.btUsageBitfield_setUsedVertexD(self.ptr, ...)
	end
	function META:SetUsedVertexC(...)
		return CLIB.btUsageBitfield_setUsedVertexC(self.ptr, ...)
	end
	function META:SetUnused4(...)
		return CLIB.btUsageBitfield_setUnused4(self.ptr, ...)
	end
	function META:SetUnused3(...)
		return CLIB.btUsageBitfield_setUnused3(self.ptr, ...)
	end
	function META:SetUnused2(...)
		return CLIB.btUsageBitfield_setUnused2(self.ptr, ...)
	end
	function META:SetUnused1(...)
		return CLIB.btUsageBitfield_setUnused1(self.ptr, ...)
	end
	function META:GetUsedVertexD(...)
		return CLIB.btUsageBitfield_getUsedVertexD(self.ptr, ...)
	end
	function META:GetUsedVertexC(...)
		return CLIB.btUsageBitfield_getUsedVertexC(self.ptr, ...)
	end
	function META:GetUsedVertexB(...)
		return CLIB.btUsageBitfield_getUsedVertexB(self.ptr, ...)
	end
	function META:GetUnused4(...)
		return CLIB.btUsageBitfield_getUnused4(self.ptr, ...)
	end
	function META:GetUnused2(...)
		return CLIB.btUsageBitfield_getUnused2(self.ptr, ...)
	end
	function META:GetUnused1(...)
		return CLIB.btUsageBitfield_getUnused1(self.ptr, ...)
	end
	function META:GetUsedVertexA(...)
		return CLIB.btUsageBitfield_getUsedVertexA(self.ptr, ...)
	end
	function META:SetUsedVertexA(...)
		return CLIB.btUsageBitfield_setUsedVertexA(self.ptr, ...)
	end
	function META:SetUsedVertexB(...)
		return CLIB.btUsageBitfield_setUsedVertexB(self.ptr, ...)
	end
	function META:Reset(...)
		return CLIB.btUsageBitfield_reset(self.ptr, ...)
	end
end
do -- btVector3_array
	local META = {}
	library.metatables.btVector3_array = META
	META.__index = function(s, k) return META[k] end
	function META:At(...)
		return CLIB.btVector3_array_at(self.ptr, ...)
	end
	function META:Set(...)
		return CLIB.btVector3_array_set(self.ptr, ...)
	end
end
do -- btSoftBody_Pose
	local META = {}
	library.metatables.btSoftBody_Pose = META
	META.__index = function(s, k) return META[k] end
	function META:SetScl(...)
		return CLIB.btSoftBody_Pose_setScl(self.ptr, ...)
	end
	function META:GetScl(...)
		return CLIB.btSoftBody_Pose_getScl(self.ptr, ...)
	end
	function META:GetPos(...)
		return CLIB.btSoftBody_Pose_getPos(self.ptr, ...)
	end
	function META:GetCom(...)
		return CLIB.btSoftBody_Pose_getCom(self.ptr, ...)
	end
	function META:GetBvolume(...)
		return CLIB.btSoftBody_Pose_getBvolume(self.ptr, ...)
	end
	function META:GetWgh(...)
		return CLIB.btSoftBody_Pose_getWgh(self.ptr, ...)
	end
	function META:GetAqq(...)
		return CLIB.btSoftBody_Pose_getAqq(self.ptr, ...)
	end
	function META:SetVolume(...)
		return CLIB.btSoftBody_Pose_setVolume(self.ptr, ...)
	end
	function META:SetRot(...)
		return CLIB.btSoftBody_Pose_setRot(self.ptr, ...)
	end
	function META:SetCom(...)
		return CLIB.btSoftBody_Pose_setCom(self.ptr, ...)
	end
	function META:SetBvolume(...)
		return CLIB.btSoftBody_Pose_setBvolume(self.ptr, ...)
	end
	function META:SetBframe(...)
		return CLIB.btSoftBody_Pose_setBframe(self.ptr, ...)
	end
	function META:SetAqq(...)
		return CLIB.btSoftBody_Pose_setAqq(self.ptr, ...)
	end
	function META:GetRot(...)
		return CLIB.btSoftBody_Pose_getRot(self.ptr, ...)
	end
	function META:GetBframe(...)
		return CLIB.btSoftBody_Pose_getBframe(self.ptr, ...)
	end
	function META:GetVolume(...)
		return CLIB.btSoftBody_Pose_getVolume(self.ptr, ...)
	end
end
do -- btConvex2dShape
	local META = {}
	library.metatables.btConvex2dShape = META
	META.__index = function(s, k) return META[k] end
	function library.CreateConvex2dShape(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btConvex2dShape_new(...)
		return self
	end
	function META:GetChildShape(...)
		return CLIB.btConvex2dShape_getChildShape(self.ptr, ...)
	end
end
do -- btGjkConvexCast
	local META = {}
	library.metatables.btGjkConvexCast = META
	META.__index = function(s, k) return META[k] end
	function library.CreateGjkConvexCast(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btGjkConvexCast_new(...)
		return self
	end
end
do -- btSoftBody_Face
	local META = {}
	library.metatables.btSoftBody_Face = META
	META.__index = function(s, k) return META[k] end
	function META:SetNormal(...)
		return CLIB.btSoftBody_Face_setNormal(self.ptr, ...)
	end
	function META:GetLeaf(...)
		return CLIB.btSoftBody_Face_getLeaf(self.ptr, ...)
	end
	function META:SetRa(...)
		return CLIB.btSoftBody_Face_setRa(self.ptr, ...)
	end
	function META:SetLeaf(...)
		return CLIB.btSoftBody_Face_setLeaf(self.ptr, ...)
	end
	function META:GetRa(...)
		return CLIB.btSoftBody_Face_getRa(self.ptr, ...)
	end
	function META:GetNormal(...)
		return CLIB.btSoftBody_Face_getNormal(self.ptr, ...)
	end
	function META:GetN(...)
		return CLIB.btSoftBody_Face_getN(self.ptr, ...)
	end
end
do -- btSoftBody_sCti
	local META = {}
	library.metatables.btSoftBody_sCti = META
	META.__index = function(s, k) return META[k] end
	function library.CreateSoftBody_sCti(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btSoftBody_sCti_new(...)
		return self
	end
	function META:GetNormal(...)
		return CLIB.btSoftBody_sCti_getNormal(self.ptr, ...)
	end
	function META:GetOffset(...)
		return CLIB.btSoftBody_sCti_getOffset(self.ptr, ...)
	end
	function META:GetColObj(...)
		return CLIB.btSoftBody_sCti_getColObj(self.ptr, ...)
	end
	function META:Delete(...)
		return CLIB.btSoftBody_sCti_delete(self.ptr, ...)
	end
	function META:SetOffset(...)
		return CLIB.btSoftBody_sCti_setOffset(self.ptr, ...)
	end
	function META:SetNormal(...)
		return CLIB.btSoftBody_sCti_setNormal(self.ptr, ...)
	end
	function META:SetColObj(...)
		return CLIB.btSoftBody_sCti_setColObj(self.ptr, ...)
	end
end
do -- btSoftBody_Node
	local META = {}
	library.metatables.btSoftBody_Node = META
	META.__index = function(s, k) return META[k] end
	function META:SetN(...)
		return CLIB.btSoftBody_Node_setN(self.ptr, ...)
	end
	function META:SetQ(...)
		return CLIB.btSoftBody_Node_setQ(self.ptr, ...)
	end
	function META:GetLeaf(...)
		return CLIB.btSoftBody_Node_getLeaf(self.ptr, ...)
	end
	function META:GetBattach(...)
		return CLIB.btSoftBody_Node_getBattach(self.ptr, ...)
	end
	function META:GetArea(...)
		return CLIB.btSoftBody_Node_getArea(self.ptr, ...)
	end
	function META:GetX(...)
		return CLIB.btSoftBody_Node_getX(self.ptr, ...)
	end
	function META:SetV(...)
		return CLIB.btSoftBody_Node_setV(self.ptr, ...)
	end
	function META:SetLeaf(...)
		return CLIB.btSoftBody_Node_setLeaf(self.ptr, ...)
	end
	function META:SetIm(...)
		return CLIB.btSoftBody_Node_setIm(self.ptr, ...)
	end
	function META:SetF(...)
		return CLIB.btSoftBody_Node_setF(self.ptr, ...)
	end
	function META:SetBattach(...)
		return CLIB.btSoftBody_Node_setBattach(self.ptr, ...)
	end
	function META:SetArea(...)
		return CLIB.btSoftBody_Node_setArea(self.ptr, ...)
	end
	function META:GetV(...)
		return CLIB.btSoftBody_Node_getV(self.ptr, ...)
	end
	function META:GetQ(...)
		return CLIB.btSoftBody_Node_getQ(self.ptr, ...)
	end
	function META:GetF(...)
		return CLIB.btSoftBody_Node_getF(self.ptr, ...)
	end
	function META:GetN(...)
		return CLIB.btSoftBody_Node_getN(self.ptr, ...)
	end
	function META:GetIm(...)
		return CLIB.btSoftBody_Node_getIm(self.ptr, ...)
	end
	function META:SetX(...)
		return CLIB.btSoftBody_Node_setX(self.ptr, ...)
	end
end
do -- btDbvt_ICollide
	local META = {}
	library.metatables.btDbvt_ICollide = META
	META.__index = function(s, k) return META[k] end
	function library.CreateDbvt_ICollide(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btDbvt_ICollide_new(...)
		return self
	end
	function META:Process(...)
		return CLIB.btDbvt_ICollide_Process(self.ptr, ...)
	end
	function META:Descent(...)
		return CLIB.btDbvt_ICollide_Descent(self.ptr, ...)
	end
	function META:AllLeaves(...)
		return CLIB.btDbvt_ICollide_AllLeaves(self.ptr, ...)
	end
	function META:Process2(...)
		return CLIB.btDbvt_ICollide_Process2(self.ptr, ...)
	end
	function META:Process3(...)
		return CLIB.btDbvt_ICollide_Process3(self.ptr, ...)
	end
	function META:Delete(...)
		return CLIB.btDbvt_ICollide_delete(self.ptr, ...)
	end
end
do -- btDantzigSolver
	local META = {}
	library.metatables.btDantzigSolver = META
	META.__index = function(s, k) return META[k] end
	function library.CreateDantzigSolver(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btDantzigSolver_new(...)
		return self
	end
end
do -- btManifoldPoint
	local META = {}
	library.metatables.btManifoldPoint = META
	META.__index = function(s, k) return META[k] end
	function library.CreateManifoldPoint(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btManifoldPoint_new2(...)
		return self
	end
	function library.CreateManifoldPoint2(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btManifoldPoint_new(...)
		return self
	end
	function META:SetAppliedImpulseLateral1(...)
		return CLIB.btManifoldPoint_setAppliedImpulseLateral1(self.ptr, ...)
	end
	function META:GetUserPersistentData(...)
		return CLIB.btManifoldPoint_getUserPersistentData(self.ptr, ...)
	end
	function META:GetCombinedContactDamping1(...)
		return CLIB.btManifoldPoint_getCombinedContactDamping1(self.ptr, ...)
	end
	function META:GetCombinedRollingFriction(...)
		return CLIB.btManifoldPoint_getCombinedRollingFriction(self.ptr, ...)
	end
	function META:Delete(...)
		return CLIB.btManifoldPoint_delete(self.ptr, ...)
	end
	function META:SetFrictionCFM(...)
		return CLIB.btManifoldPoint_setFrictionCFM(self.ptr, ...)
	end
	function META:GetCombinedFriction(...)
		return CLIB.btManifoldPoint_getCombinedFriction(self.ptr, ...)
	end
	function META:GetAppliedImpulse(...)
		return CLIB.btManifoldPoint_getAppliedImpulse(self.ptr, ...)
	end
	function META:SetUserPersistentData(...)
		return CLIB.btManifoldPoint_setUserPersistentData(self.ptr, ...)
	end
	function META:SetCombinedRestitution(...)
		return CLIB.btManifoldPoint_setCombinedRestitution(self.ptr, ...)
	end
	function META:GetNormalWorldOnB(...)
		return CLIB.btManifoldPoint_getNormalWorldOnB(self.ptr, ...)
	end
	function META:SetCombinedFriction(...)
		return CLIB.btManifoldPoint_setCombinedFriction(self.ptr, ...)
	end
	function META:SetIndex0(...)
		return CLIB.btManifoldPoint_setIndex0(self.ptr, ...)
	end
	function META:SetLateralFrictionDir2(...)
		return CLIB.btManifoldPoint_setLateralFrictionDir2(self.ptr, ...)
	end
	function META:SetLateralFrictionDir1(...)
		return CLIB.btManifoldPoint_setLateralFrictionDir1(self.ptr, ...)
	end
	function META:GetLateralFrictionDir2(...)
		return CLIB.btManifoldPoint_getLateralFrictionDir2(self.ptr, ...)
	end
	function META:SetIndex1(...)
		return CLIB.btManifoldPoint_setIndex1(self.ptr, ...)
	end
	function META:GetCombinedRestitution(...)
		return CLIB.btManifoldPoint_getCombinedRestitution(self.ptr, ...)
	end
	function META:SetContactMotion1(...)
		return CLIB.btManifoldPoint_setContactMotion1(self.ptr, ...)
	end
	function META:GetPositionWorldOnB(...)
		return CLIB.btManifoldPoint_getPositionWorldOnB(self.ptr, ...)
	end
	function META:SetContactMotion2(...)
		return CLIB.btManifoldPoint_setContactMotion2(self.ptr, ...)
	end
	function META:GetContactMotion1(...)
		return CLIB.btManifoldPoint_getContactMotion1(self.ptr, ...)
	end
	function META:GetDistance1(...)
		return CLIB.btManifoldPoint_getDistance1(self.ptr, ...)
	end
	function META:SetPositionWorldOnA(...)
		return CLIB.btManifoldPoint_setPositionWorldOnA(self.ptr, ...)
	end
	function META:GetContactMotion2(...)
		return CLIB.btManifoldPoint_getContactMotion2(self.ptr, ...)
	end
	function META:GetPositionWorldOnA(...)
		return CLIB.btManifoldPoint_getPositionWorldOnA(self.ptr, ...)
	end
	function META:SetContactPointFlags(...)
		return CLIB.btManifoldPoint_setContactPointFlags(self.ptr, ...)
	end
	function META:SetCombinedContactDamping1(...)
		return CLIB.btManifoldPoint_setCombinedContactDamping1(self.ptr, ...)
	end
	function META:GetLateralFrictionDir1(...)
		return CLIB.btManifoldPoint_getLateralFrictionDir1(self.ptr, ...)
	end
	function META:SetCombinedContactStiffness1(...)
		return CLIB.btManifoldPoint_setCombinedContactStiffness1(self.ptr, ...)
	end
	function META:GetPartId1(...)
		return CLIB.btManifoldPoint_getPartId1(self.ptr, ...)
	end
	function META:GetLocalPointA(...)
		return CLIB.btManifoldPoint_getLocalPointA(self.ptr, ...)
	end
	function META:GetContactPointFlags(...)
		return CLIB.btManifoldPoint_getContactPointFlags(self.ptr, ...)
	end
	function META:SetLocalPointA(...)
		return CLIB.btManifoldPoint_setLocalPointA(self.ptr, ...)
	end
	function META:SetLifeTime(...)
		return CLIB.btManifoldPoint_setLifeTime(self.ptr, ...)
	end
	function META:SetAppliedImpulse(...)
		return CLIB.btManifoldPoint_setAppliedImpulse(self.ptr, ...)
	end
	function META:SetLocalPointB(...)
		return CLIB.btManifoldPoint_setLocalPointB(self.ptr, ...)
	end
	function META:GetLocalPointB(...)
		return CLIB.btManifoldPoint_getLocalPointB(self.ptr, ...)
	end
	function META:SetPartId1(...)
		return CLIB.btManifoldPoint_setPartId1(self.ptr, ...)
	end
	function META:SetCombinedRollingFriction(...)
		return CLIB.btManifoldPoint_setCombinedRollingFriction(self.ptr, ...)
	end
	function META:GetLifeTime(...)
		return CLIB.btManifoldPoint_getLifeTime(self.ptr, ...)
	end
	function META:GetContactERP(...)
		return CLIB.btManifoldPoint_getContactERP(self.ptr, ...)
	end
	function META:SetPartId0(...)
		return CLIB.btManifoldPoint_setPartId0(self.ptr, ...)
	end
	function META:GetAppliedImpulseLateral2(...)
		return CLIB.btManifoldPoint_getAppliedImpulseLateral2(self.ptr, ...)
	end
	function META:SetContactCFM(...)
		return CLIB.btManifoldPoint_setContactCFM(self.ptr, ...)
	end
	function META:GetIndex1(...)
		return CLIB.btManifoldPoint_getIndex1(self.ptr, ...)
	end
	function META:GetFrictionCFM(...)
		return CLIB.btManifoldPoint_getFrictionCFM(self.ptr, ...)
	end
	function META:GetDistance(...)
		return CLIB.btManifoldPoint_getDistance(self.ptr, ...)
	end
	function META:GetContactCFM(...)
		return CLIB.btManifoldPoint_getContactCFM(self.ptr, ...)
	end
	function META:SetNormalWorldOnB(...)
		return CLIB.btManifoldPoint_setNormalWorldOnB(self.ptr, ...)
	end
	function META:SetDistance1(...)
		return CLIB.btManifoldPoint_setDistance1(self.ptr, ...)
	end
	function META:GetPartId0(...)
		return CLIB.btManifoldPoint_getPartId0(self.ptr, ...)
	end
	function META:GetCombinedContactStiffness1(...)
		return CLIB.btManifoldPoint_getCombinedContactStiffness1(self.ptr, ...)
	end
	function META:SetContactERP(...)
		return CLIB.btManifoldPoint_setContactERP(self.ptr, ...)
	end
	function META:GetAppliedImpulseLateral1(...)
		return CLIB.btManifoldPoint_getAppliedImpulseLateral1(self.ptr, ...)
	end
	function META:GetIndex0(...)
		return CLIB.btManifoldPoint_getIndex0(self.ptr, ...)
	end
	function META:SetDistance(...)
		return CLIB.btManifoldPoint_setDistance(self.ptr, ...)
	end
	function META:SetPositionWorldOnB(...)
		return CLIB.btManifoldPoint_setPositionWorldOnB(self.ptr, ...)
	end
	function META:SetAppliedImpulseLateral2(...)
		return CLIB.btManifoldPoint_setAppliedImpulseLateral2(self.ptr, ...)
	end
end
do -- btChunk_setDna
	local META = {}
	library.metatables.btChunk_setDna = META
	META.__index = function(s, k) return META[k] end
	function META:Nr(...)
		return CLIB.btChunk_setDna_nr(self.ptr, ...)
	end
end
do -- btTriangleInfo
	local META = {}
	library.metatables.btTriangleInfo = META
	META.__index = function(s, k) return META[k] end
	function library.CreateTriangleInfo(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btTriangleInfo_new(...)
		return self
	end
	function META:SetEdgeV1V2Angle(...)
		return CLIB.btTriangleInfo_setEdgeV1V2Angle(self.ptr, ...)
	end
	function META:Delete(...)
		return CLIB.btTriangleInfo_delete(self.ptr, ...)
	end
	function META:SetFlags(...)
		return CLIB.btTriangleInfo_setFlags(self.ptr, ...)
	end
	function META:SetEdgeV2V0Angle(...)
		return CLIB.btTriangleInfo_setEdgeV2V0Angle(self.ptr, ...)
	end
	function META:GetFlags(...)
		return CLIB.btTriangleInfo_getFlags(self.ptr, ...)
	end
	function META:GetEdgeV2V0Angle(...)
		return CLIB.btTriangleInfo_getEdgeV2V0Angle(self.ptr, ...)
	end
	function META:GetEdgeV1V2Angle(...)
		return CLIB.btTriangleInfo_getEdgeV1V2Angle(self.ptr, ...)
	end
	function META:GetEdgeV0V1Angle(...)
		return CLIB.btTriangleInfo_getEdgeV0V1Angle(self.ptr, ...)
	end
	function META:SetEdgeV0V1Angle(...)
		return CLIB.btTriangleInfo_setEdgeV0V1Angle(self.ptr, ...)
	end
end
do -- btCapsuleShape
	local META = {}
	library.metatables.btCapsuleShape = META
	META.__index = function(s, k) return META[k] end
	function library.CreateCapsuleShape(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btCapsuleShape_new(...)
		return self
	end
	function META:DeSerializeFloat(...)
		return CLIB.btCapsuleShape_deSerializeFloat(self.ptr, ...)
	end
	function META:GetUpAxis(...)
		return CLIB.btCapsuleShape_getUpAxis(self.ptr, ...)
	end
	function META:GetHalfHeight(...)
		return CLIB.btCapsuleShape_getHalfHeight(self.ptr, ...)
	end
	function META:GetRadius(...)
		return CLIB.btCapsuleShape_getRadius(self.ptr, ...)
	end
end
do -- btChunk_getDna
	local META = {}
	library.metatables.btChunk_getDna = META
	META.__index = function(s, k) return META[k] end
	function META:Nr(...)
		return CLIB.btChunk_getDna_nr(self.ptr, ...)
	end
end
do -- btDbvt_IWriter
	local META = {}
	library.metatables.btDbvt_IWriter = META
	META.__index = function(s, k) return META[k] end
	function META:WriteLeaf(...)
		return CLIB.btDbvt_IWriter_WriteLeaf(self.ptr, ...)
	end
	function META:Prepare(...)
		return CLIB.btDbvt_IWriter_Prepare(self.ptr, ...)
	end
	function META:WriteNode(...)
		return CLIB.btDbvt_IWriter_WriteNode(self.ptr, ...)
	end
	function META:Delete(...)
		return CLIB.btDbvt_IWriter_delete(self.ptr, ...)
	end
end
do -- btDbvt_sStkCLN
	local META = {}
	library.metatables.btDbvt_sStkCLN = META
	META.__index = function(s, k) return META[k] end
	function library.CreateDbvt_sStkCLN(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btDbvt_sStkCLN_new(...)
		return self
	end
	function META:GetParent(...)
		return CLIB.btDbvt_sStkCLN_getParent(self.ptr, ...)
	end
	function META:Delete(...)
		return CLIB.btDbvt_sStkCLN_delete(self.ptr, ...)
	end
	function META:SetNode(...)
		return CLIB.btDbvt_sStkCLN_setNode(self.ptr, ...)
	end
	function META:SetParent(...)
		return CLIB.btDbvt_sStkCLN_setParent(self.ptr, ...)
	end
	function META:GetNode(...)
		return CLIB.btDbvt_sStkCLN_getNode(self.ptr, ...)
	end
end
do -- btAABB_collide
	local META = {}
	library.metatables.btAABB_collide = META
	META.__index = function(s, k) return META[k] end
	function META:Ray(...)
		return CLIB.btAABB_collide_ray(self.ptr, ...)
	end
	function META:Plane(...)
		return CLIB.btAABB_collide_plane(self.ptr, ...)
	end
end
do -- btTriangleMesh
	local META = {}
	library.metatables.btTriangleMesh = META
	META.__index = function(s, k) return META[k] end
	function library.CreateTriangleMesh(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btTriangleMesh_new(...)
		return self
	end
	function META:GetUse4componentVertices(...)
		return CLIB.btTriangleMesh_getUse4componentVertices(self.ptr, ...)
	end
	function META:AddIndex(...)
		return CLIB.btTriangleMesh_addIndex(self.ptr, ...)
	end
	function META:FindOrAddVertex(...)
		return CLIB.btTriangleMesh_findOrAddVertex(self.ptr, ...)
	end
	function META:AddTriangleIndices(...)
		return CLIB.btTriangleMesh_addTriangleIndices(self.ptr, ...)
	end
	function META:GetNumTriangles(...)
		return CLIB.btTriangleMesh_getNumTriangles(self.ptr, ...)
	end
	function META:GetWeldingThreshold(...)
		return CLIB.btTriangleMesh_getWeldingThreshold(self.ptr, ...)
	end
	function META:GetUse32bitIndices(...)
		return CLIB.btTriangleMesh_getUse32bitIndices(self.ptr, ...)
	end
	function META:SetWeldingThreshold(...)
		return CLIB.btTriangleMesh_setWeldingThreshold(self.ptr, ...)
	end
	function META:AddTriangle(...)
		return CLIB.btTriangleMesh_addTriangle(self.ptr, ...)
	end
end
do -- btOptimizedBvh
	local META = {}
	library.metatables.btOptimizedBvh = META
	META.__index = function(s, k) return META[k] end
	function library.CreateOptimizedBvh(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btOptimizedBvh_new(...)
		return self
	end
	function META:Build(...)
		return CLIB.btOptimizedBvh_build(self.ptr, ...)
	end
	function META:SerializeInPlace(...)
		return CLIB.btOptimizedBvh_serializeInPlace(self.ptr, ...)
	end
	function META:DeSerializeInPlace(...)
		return CLIB.btOptimizedBvh_deSerializeInPlace(self.ptr, ...)
	end
	function META:UpdateBvhNodes(...)
		return CLIB.btOptimizedBvh_updateBvhNodes(self.ptr, ...)
	end
	function META:Refit(...)
		return CLIB.btOptimizedBvh_refit(self.ptr, ...)
	end
	function META:RefitPartial(...)
		return CLIB.btOptimizedBvh_refitPartial(self.ptr, ...)
	end
end
do -- btConcaveShape
	local META = {}
	library.metatables.btConcaveShape = META
	META.__index = function(s, k) return META[k] end
	function META:ProcessAllTriangles(...)
		return CLIB.btConcaveShape_processAllTriangles(self.ptr, ...)
	end
end
do -- btPairSet_push
	local META = {}
	library.metatables.btPairSet_push = META
	META.__index = function(s, k) return META[k] end
end
do -- btDbvt_sStkNPS
	local META = {}
	library.metatables.btDbvt_sStkNPS = META
	META.__index = function(s, k) return META[k] end
	function library.CreateDbvt_sStkNPS(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btDbvt_sStkNPS_new(...)
		return self
	end
	function library.CreateDbvt_sStkNPS2(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btDbvt_sStkNPS_new2(...)
		return self
	end
	function META:Delete(...)
		return CLIB.btDbvt_sStkNPS_delete(self.ptr, ...)
	end
	function META:SetMask(...)
		return CLIB.btDbvt_sStkNPS_setMask(self.ptr, ...)
	end
	function META:SetNode(...)
		return CLIB.btDbvt_sStkNPS_setNode(self.ptr, ...)
	end
	function META:GetNode(...)
		return CLIB.btDbvt_sStkNPS_getNode(self.ptr, ...)
	end
	function META:GetValue(...)
		return CLIB.btDbvt_sStkNPS_getValue(self.ptr, ...)
	end
	function META:SetValue(...)
		return CLIB.btDbvt_sStkNPS_setValue(self.ptr, ...)
	end
	function META:GetMask(...)
		return CLIB.btDbvt_sStkNPS_getMask(self.ptr, ...)
	end
end
do -- btQuantizedBvh
	local META = {}
	library.metatables.btQuantizedBvh = META
	META.__index = function(s, k) return META[k] end
	function library.CreateQuantizedBvh(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btQuantizedBvh_new(...)
		return self
	end
	function META:GetLeafNodeArray(...)
		return CLIB.btQuantizedBvh_getLeafNodeArray(self.ptr, ...)
	end
	function META:ReportRayOverlappingNodex(...)
		return CLIB.btQuantizedBvh_reportRayOverlappingNodex(self.ptr, ...)
	end
	function META:Delete(...)
		return CLIB.btQuantizedBvh_delete(self.ptr, ...)
	end
	function META:CalculateSerializeBufferSizeNew(...)
		return CLIB.btQuantizedBvh_calculateSerializeBufferSizeNew(self.ptr, ...)
	end
	function META:IsQuantized(...)
		return CLIB.btQuantizedBvh_isQuantized(self.ptr, ...)
	end
	function META:CalculateSerializeBufferSize(...)
		return CLIB.btQuantizedBvh_calculateSerializeBufferSize(self.ptr, ...)
	end
	function META:DeSerializeFloat(...)
		return CLIB.btQuantizedBvh_deSerializeFloat(self.ptr, ...)
	end
	function META:Serialize(...)
		return CLIB.btQuantizedBvh_serialize(self.ptr, ...)
	end
	function META:DeSerializeDouble(...)
		return CLIB.btQuantizedBvh_deSerializeDouble(self.ptr, ...)
	end
	function META:UnQuantize(...)
		return CLIB.btQuantizedBvh_unQuantize(self.ptr, ...)
	end
	function META:SetTraversalMode(...)
		return CLIB.btQuantizedBvh_setTraversalMode(self.ptr, ...)
	end
	function META:Serialize2(...)
		return CLIB.btQuantizedBvh_serialize2(self.ptr, ...)
	end
	function META:ReportBoxCastOverlappingNodex(...)
		return CLIB.btQuantizedBvh_reportBoxCastOverlappingNodex(self.ptr, ...)
	end
	function META:ReportAabbOverlappingNodex(...)
		return CLIB.btQuantizedBvh_reportAabbOverlappingNodex(self.ptr, ...)
	end
	function META:Quantize(...)
		return CLIB.btQuantizedBvh_quantize(self.ptr, ...)
	end
	function META:GetSubtreeInfoArray(...)
		return CLIB.btQuantizedBvh_getSubtreeInfoArray(self.ptr, ...)
	end
	function META:GetQuantizedNodeArray(...)
		return CLIB.btQuantizedBvh_getQuantizedNodeArray(self.ptr, ...)
	end
	function META:GetAlignmentSerializationPadding(...)
		return CLIB.btQuantizedBvh_getAlignmentSerializationPadding(self.ptr, ...)
	end
	function META:DeSerializeInPlace(...)
		return CLIB.btQuantizedBvh_deSerializeInPlace(self.ptr, ...)
	end
	function META:BuildInternal(...)
		return CLIB.btQuantizedBvh_buildInternal(self.ptr, ...)
	end
	function META:QuantizeWithClamp(...)
		return CLIB.btQuantizedBvh_quantizeWithClamp(self.ptr, ...)
	end
	function META:SetQuantizationValues(...)
		return CLIB.btQuantizedBvh_setQuantizationValues(self.ptr, ...)
	end
end
do -- btAngularLimit
	local META = {}
	library.metatables.btAngularLimit = META
	META.__index = function(s, k) return META[k] end
	function library.CreateAngularLimit(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btAngularLimit_new(...)
		return self
	end
	function META:GetBiasFactor(...)
		return CLIB.btAngularLimit_getBiasFactor(self.ptr, ...)
	end
	function META:GetError(...)
		return CLIB.btAngularLimit_getError(self.ptr, ...)
	end
	function META:Fit(...)
		return CLIB.btAngularLimit_fit(self.ptr, ...)
	end
	function META:Test(...)
		return CLIB.btAngularLimit_test(self.ptr, ...)
	end
	function META:GetCorrection(...)
		return CLIB.btAngularLimit_getCorrection(self.ptr, ...)
	end
	function META:IsLimit(...)
		return CLIB.btAngularLimit_isLimit(self.ptr, ...)
	end
	function META:GetSign(...)
		return CLIB.btAngularLimit_getSign(self.ptr, ...)
	end
	function META:Delete(...)
		return CLIB.btAngularLimit_delete(self.ptr, ...)
	end
	function META:GetHigh(...)
		return CLIB.btAngularLimit_getHigh(self.ptr, ...)
	end
	function META:GetRelaxationFactor(...)
		return CLIB.btAngularLimit_getRelaxationFactor(self.ptr, ...)
	end
	function META:GetSoftness(...)
		return CLIB.btAngularLimit_getSoftness(self.ptr, ...)
	end
	function META:GetHalfRange(...)
		return CLIB.btAngularLimit_getHalfRange(self.ptr, ...)
	end
	function META:Set(...)
		return CLIB.btAngularLimit_set(self.ptr, ...)
	end
	function META:GetLow(...)
		return CLIB.btAngularLimit_getLow(self.ptr, ...)
	end
end
do -- btMotionState
	local META = {}
	library.metatables.btMotionState = META
	META.__index = function(s, k) return META[k] end
	function META:SetWorldTransform(...)
		return CLIB.btMotionState_setWorldTransform(self.ptr, ...)
	end
	function META:GetWorldTransform(...)
		return CLIB.btMotionState_getWorldTransform(self.ptr, ...)
	end
	function META:Delete(...)
		return CLIB.btMotionState_delete(self.ptr, ...)
	end
end
do -- btSphereShape
	local META = {}
	library.metatables.btSphereShape = META
	function META:__index(k)
		local v

		v = META[k]
		if v ~= nil then
			return v
		end
		v = library.metatables.btConvexInternalShape.__index(self, k)
		if v ~= nil then
			return v
		end
	end
	function library.CreateSphereShape(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btSphereShape_new(...)
		return self
	end
	function META:GetRadius(...)
		return CLIB.btSphereShape_getRadius(self.ptr, ...)
	end
	function META:SetUnscaledRadius(...)
		return CLIB.btSphereShape_setUnscaledRadius(self.ptr, ...)
	end
end
do -- btDbvt_sStkNN
	local META = {}
	library.metatables.btDbvt_sStkNN = META
	META.__index = function(s, k) return META[k] end
	function library.CreateDbvt_sStkNN(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btDbvt_sStkNN_new(...)
		return self
	end
	function library.CreateDbvt_sStkNN2(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btDbvt_sStkNN_new2(...)
		return self
	end
	function META:Delete(...)
		return CLIB.btDbvt_sStkNN_delete(self.ptr, ...)
	end
	function META:SetB(...)
		return CLIB.btDbvt_sStkNN_setB(self.ptr, ...)
	end
	function META:SetA(...)
		return CLIB.btDbvt_sStkNN_setA(self.ptr, ...)
	end
	function META:GetA(...)
		return CLIB.btDbvt_sStkNN_getA(self.ptr, ...)
	end
	function META:GetB(...)
		return CLIB.btDbvt_sStkNN_getB(self.ptr, ...)
	end
end
do -- btDbvt_IClone
	local META = {}
	library.metatables.btDbvt_IClone = META
	META.__index = function(s, k) return META[k] end
	function library.CreateDbvt_IClone(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btDbvt_IClone_new(...)
		return self
	end
	function META:CloneLeaf(...)
		return CLIB.btDbvt_IClone_CloneLeaf(self.ptr, ...)
	end
	function META:Delete(...)
		return CLIB.btDbvt_IClone_delete(self.ptr, ...)
	end
end
do -- btGhostObject
	local META = {}
	library.metatables.btGhostObject = META
	META.__index = function(s, k) return META[k] end
	function library.CreateGhostObject(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btGhostObject_new(...)
		return self
	end
	function META:ConvexSweepTest(...)
		return CLIB.btGhostObject_convexSweepTest(self.ptr, ...)
	end
	function META:GetOverlappingObject(...)
		return CLIB.btGhostObject_getOverlappingObject(self.ptr, ...)
	end
	function META:RayTest(...)
		return CLIB.btGhostObject_rayTest(self.ptr, ...)
	end
	function META:RemoveOverlappingObjectInternal(...)
		return CLIB.btGhostObject_removeOverlappingObjectInternal(self.ptr, ...)
	end
	function META:GetNumOverlappingObjects(...)
		return CLIB.btGhostObject_getNumOverlappingObjects(self.ptr, ...)
	end
	function META:AddOverlappingObjectInternal(...)
		return CLIB.btGhostObject_addOverlappingObjectInternal(self.ptr, ...)
	end
	function META:Upcast(...)
		return CLIB.btGhostObject_upcast(self.ptr, ...)
	end
	function META:GetOverlappingPairs(...)
		return CLIB.btGhostObject_getOverlappingPairs(self.ptr, ...)
	end
end
do -- btConvexShape
	local META = {}
	library.metatables.btConvexShape = META
	function META:__index(k)
		local v

		v = META[k]
		if v ~= nil then
			return v
		end
		v = library.metatables.btCollisionShape.__index(self, k)
		if v ~= nil then
			return v
		end
	end
	function META:GetNumPreferredPenetrationDirections(...)
		return CLIB.btConvexShape_getNumPreferredPenetrationDirections(self.ptr, ...)
	end
	function META:BatchedUnitVectorGetSupportingVertexWithoutMargin(...)
		return CLIB.btConvexShape_batchedUnitVectorGetSupportingVertexWithoutMargin(self.ptr, ...)
	end
	function META:LocalGetSupportVertexWithoutMarginNonVirtual(...)
		return CLIB.btConvexShape_localGetSupportVertexWithoutMarginNonVirtual(self.ptr, ...)
	end
	function META:GetPreferredPenetrationDirection(...)
		return CLIB.btConvexShape_getPreferredPenetrationDirection(self.ptr, ...)
	end
	function META:Project(...)
		return CLIB.btConvexShape_project(self.ptr, ...)
	end
	function META:LocalGetSupportingVertexWithoutMargin(...)
		return CLIB.btConvexShape_localGetSupportingVertexWithoutMargin(self.ptr, ...)
	end
	function META:GetAabbNonVirtual(...)
		return CLIB.btConvexShape_getAabbNonVirtual(self.ptr, ...)
	end
	function META:GetMarginNonVirtual(...)
		return CLIB.btConvexShape_getMarginNonVirtual(self.ptr, ...)
	end
	function META:GetAabbSlow(...)
		return CLIB.btConvexShape_getAabbSlow(self.ptr, ...)
	end
	function META:LocalGetSupportVertexNonVirtual(...)
		return CLIB.btConvexShape_localGetSupportVertexNonVirtual(self.ptr, ...)
	end
	function META:LocalGetSupportingVertex(...)
		return CLIB.btConvexShape_localGetSupportingVertex(self.ptr, ...)
	end
end
do -- btIndexedMesh
	local META = {}
	library.metatables.btIndexedMesh = META
	META.__index = function(s, k) return META[k] end
	function library.CreateIndexedMesh(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btIndexedMesh_new(...)
		return self
	end
	function META:GetIndexType(...)
		return CLIB.btIndexedMesh_getIndexType(self.ptr, ...)
	end
	function META:Delete(...)
		return CLIB.btIndexedMesh_delete(self.ptr, ...)
	end
	function META:GetNumVertices(...)
		return CLIB.btIndexedMesh_getNumVertices(self.ptr, ...)
	end
	function META:SetVertexStride(...)
		return CLIB.btIndexedMesh_setVertexStride(self.ptr, ...)
	end
	function META:SetTriangleIndexStride(...)
		return CLIB.btIndexedMesh_setTriangleIndexStride(self.ptr, ...)
	end
	function META:SetTriangleIndexBase(...)
		return CLIB.btIndexedMesh_setTriangleIndexBase(self.ptr, ...)
	end
	function META:SetNumVertices(...)
		return CLIB.btIndexedMesh_setNumVertices(self.ptr, ...)
	end
	function META:SetNumTriangles(...)
		return CLIB.btIndexedMesh_setNumTriangles(self.ptr, ...)
	end
	function META:GetVertexBase(...)
		return CLIB.btIndexedMesh_getVertexBase(self.ptr, ...)
	end
	function META:GetTriangleIndexStride(...)
		return CLIB.btIndexedMesh_getTriangleIndexStride(self.ptr, ...)
	end
	function META:GetTriangleIndexBase(...)
		return CLIB.btIndexedMesh_getTriangleIndexBase(self.ptr, ...)
	end
	function META:GetNumTriangles(...)
		return CLIB.btIndexedMesh_getNumTriangles(self.ptr, ...)
	end
	function META:GetVertexStride(...)
		return CLIB.btIndexedMesh_getVertexStride(self.ptr, ...)
	end
	function META:SetVertexType(...)
		return CLIB.btIndexedMesh_setVertexType(self.ptr, ...)
	end
	function META:SetIndexType(...)
		return CLIB.btIndexedMesh_setIndexType(self.ptr, ...)
	end
	function META:GetVertexType(...)
		return CLIB.btIndexedMesh_getVertexType(self.ptr, ...)
	end
	function META:SetVertexBase(...)
		return CLIB.btIndexedMesh_setVertexBase(self.ptr, ...)
	end
end
do -- btDbvt_sStkNP
	local META = {}
	library.metatables.btDbvt_sStkNP = META
	META.__index = function(s, k) return META[k] end
	function library.CreateDbvt_sStkNP(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btDbvt_sStkNP_new(...)
		return self
	end
	function META:GetNode(...)
		return CLIB.btDbvt_sStkNP_getNode(self.ptr, ...)
	end
	function META:SetMask(...)
		return CLIB.btDbvt_sStkNP_setMask(self.ptr, ...)
	end
	function META:Delete(...)
		return CLIB.btDbvt_sStkNP_delete(self.ptr, ...)
	end
	function META:SetNode(...)
		return CLIB.btDbvt_sStkNP_setNode(self.ptr, ...)
	end
	function META:GetMask(...)
		return CLIB.btDbvt_sStkNP_getMask(self.ptr, ...)
	end
end
do -- btConeShapeX
	local META = {}
	library.metatables.btConeShapeX = META
	META.__index = function(s, k) return META[k] end
	function library.CreateConeShapeX(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btConeShapeX_new(...)
		return self
	end
end
do -- btGImpactBvh
	local META = {}
	library.metatables.btGImpactBvh = META
	META.__index = function(s, k) return META[k] end
	function library.CreateGImpactBvh(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btGImpactBvh_new2(...)
		return self
	end
	function library.CreateGImpactBvh2(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btGImpactBvh_new(...)
		return self
	end
	function META:GetEscapeNodeIndex(...)
		return CLIB.btGImpactBvh_getEscapeNodeIndex(self.ptr, ...)
	end
	function META:GetGlobalBox(...)
		return CLIB.btGImpactBvh_getGlobalBox(self.ptr, ...)
	end
	function META:GetLeftNode(...)
		return CLIB.btGImpactBvh_getLeftNode(self.ptr, ...)
	end
	function META:SetNodeBound(...)
		return CLIB.btGImpactBvh_setNodeBound(self.ptr, ...)
	end
	function META:Delete(...)
		return CLIB.btGImpactBvh_delete(self.ptr, ...)
	end
	function META:BuildSet(...)
		return CLIB.btGImpactBvh_buildSet(self.ptr, ...)
	end
	function META:SetPrimitiveManager(...)
		return CLIB.btGImpactBvh_setPrimitiveManager(self.ptr, ...)
	end
	function META:GetNodeCount(...)
		return CLIB.btGImpactBvh_getNodeCount(self.ptr, ...)
	end
	function META:GetNodeData(...)
		return CLIB.btGImpactBvh_getNodeData(self.ptr, ...)
	end
	function META:IsLeafNode(...)
		return CLIB.btGImpactBvh_isLeafNode(self.ptr, ...)
	end
	function META:BoxQuery(...)
		return CLIB.btGImpactBvh_boxQuery(self.ptr, ...)
	end
	function META:HasHierarchy(...)
		return CLIB.btGImpactBvh_hasHierarchy(self.ptr, ...)
	end
	function META:BoxQueryTrans(...)
		return CLIB.btGImpactBvh_boxQueryTrans(self.ptr, ...)
	end
	function META:GetNodeTriangle(...)
		return CLIB.btGImpactBvh_getNodeTriangle(self.ptr, ...)
	end
	function META:GetPrimitiveManager(...)
		return CLIB.btGImpactBvh_getPrimitiveManager(self.ptr, ...)
	end
	function META:Update(...)
		return CLIB.btGImpactBvh_update(self.ptr, ...)
	end
	function META:IsTrimesh(...)
		return CLIB.btGImpactBvh_isTrimesh(self.ptr, ...)
	end
	function META:GetRightNode(...)
		return CLIB.btGImpactBvh_getRightNode(self.ptr, ...)
	end
	function META:RayQuery(...)
		return CLIB.btGImpactBvh_rayQuery(self.ptr, ...)
	end
	function META:GetNodeBound(...)
		return CLIB.btGImpactBvh_getNodeBound(self.ptr, ...)
	end
end
do -- btBox2dShape
	local META = {}
	library.metatables.btBox2dShape = META
	META.__index = function(s, k) return META[k] end
	function library.CreateBox2dShape(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btBox2dShape_new2(...)
		return self
	end
	function library.CreateBox2dShape2(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btBox2dShape_new3(...)
		return self
	end
	function library.CreateBox2dShape3(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btBox2dShape_new(...)
		return self
	end
	function META:GetPlaneEquation(...)
		return CLIB.btBox2dShape_getPlaneEquation(self.ptr, ...)
	end
	function META:GetNormals(...)
		return CLIB.btBox2dShape_getNormals(self.ptr, ...)
	end
	function META:GetHalfExtentsWithMargin(...)
		return CLIB.btBox2dShape_getHalfExtentsWithMargin(self.ptr, ...)
	end
	function META:GetVertexCount(...)
		return CLIB.btBox2dShape_getVertexCount(self.ptr, ...)
	end
	function META:GetCentroid(...)
		return CLIB.btBox2dShape_getCentroid(self.ptr, ...)
	end
	function META:GetVertices(...)
		return CLIB.btBox2dShape_getVertices(self.ptr, ...)
	end
	function META:GetHalfExtentsWithoutMargin(...)
		return CLIB.btBox2dShape_getHalfExtentsWithoutMargin(self.ptr, ...)
	end
end
do -- btSerializer
	local META = {}
	library.metatables.btSerializer = META
	META.__index = function(s, k) return META[k] end
	function META:Delete(...)
		return CLIB.btSerializer_delete(self.ptr, ...)
	end
end
do -- btIDebugDraw
	local META = {}
	library.metatables.btIDebugDraw = META
	META.__index = function(s, k) return META[k] end
	function META:Delete(...)
		return CLIB.btIDebugDraw_delete(self.ptr, ...)
	end
end
do -- btDispatcher
	local META = {}
	library.metatables.btDispatcher = META
	META.__index = function(s, k) return META[k] end
	function META:GetNumManifolds(...)
		return CLIB.btDispatcher_getNumManifolds(self.ptr, ...)
	end
	function META:GetInternalManifoldPointer(...)
		return CLIB.btDispatcher_getInternalManifoldPointer(self.ptr, ...)
	end
	function META:FreeCollisionAlgorithm(...)
		return CLIB.btDispatcher_freeCollisionAlgorithm(self.ptr, ...)
	end
	function META:GetManifoldByIndexInternal(...)
		return CLIB.btDispatcher_getManifoldByIndexInternal(self.ptr, ...)
	end
	function META:ReleaseManifold(...)
		return CLIB.btDispatcher_releaseManifold(self.ptr, ...)
	end
	function META:GetNewManifold(...)
		return CLIB.btDispatcher_getNewManifold(self.ptr, ...)
	end
	function META:NeedsCollision(...)
		return CLIB.btDispatcher_needsCollision(self.ptr, ...)
	end
	function META:DispatchAllCollisionPairs(...)
		return CLIB.btDispatcher_dispatchAllCollisionPairs(self.ptr, ...)
	end
	function META:Delete(...)
		return CLIB.btDispatcher_delete(self.ptr, ...)
	end
	function META:GetInternalManifoldPool(...)
		return CLIB.btDispatcher_getInternalManifoldPool(self.ptr, ...)
	end
	function META:ClearManifold(...)
		return CLIB.btDispatcher_clearManifold(self.ptr, ...)
	end
	function META:NeedsResponse(...)
		return CLIB.btDispatcher_needsResponse(self.ptr, ...)
	end
	function META:AllocateCollisionAlgorithm(...)
		return CLIB.btDispatcher_allocateCollisionAlgorithm(self.ptr, ...)
	end
	function META:FindAlgorithm(...)
		return CLIB.btDispatcher_findAlgorithm(self.ptr, ...)
	end
end
do -- btDbvt_array
	local META = {}
	library.metatables.btDbvt_array = META
	META.__index = function(s, k) return META[k] end
	function META:At(...)
		return CLIB.btDbvt_array_at(self.ptr, ...)
	end
end
do -- btMLCPSolver
	local META = {}
	library.metatables.btMLCPSolver = META
	META.__index = function(s, k) return META[k] end
	function library.CreateMLCPSolver(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btMLCPSolver_new(...)
		return self
	end
	function META:SetMLCPSolver(...)
		return CLIB.btMLCPSolver_setMLCPSolver(self.ptr, ...)
	end
	function META:GetNumFallbacks(...)
		return CLIB.btMLCPSolver_getNumFallbacks(self.ptr, ...)
	end
	function META:SetNumFallbacks(...)
		return CLIB.btMLCPSolver_setNumFallbacks(self.ptr, ...)
	end
end
do -- btEmptyShape
	local META = {}
	library.metatables.btEmptyShape = META
	META.__index = function(s, k) return META[k] end
	function library.CreateEmptyShape(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btEmptyShape_new(...)
		return self
	end
end
do -- btSparseSdf3
	local META = {}
	library.metatables.btSparseSdf3 = META
	META.__index = function(s, k) return META[k] end
	function META:GarbageCollect(...)
		return CLIB.btSparseSdf3_GarbageCollect(self.ptr, ...)
	end
	function META:Reset(...)
		return CLIB.btSparseSdf3_Reset(self.ptr, ...)
	end
	function META:RemoveReferences(...)
		return CLIB.btSparseSdf3_RemoveReferences(self.ptr, ...)
	end
	function META:Initialize(...)
		return CLIB.btSparseSdf3_Initialize(self.ptr, ...)
	end
end
do -- btDbvtAabbMm
	local META = {}
	library.metatables.btDbvtAabbMm = META
	META.__index = function(s, k) return META[k] end
	function library.CreateDbvtAabbMm(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btDbvtAabbMm_new(...)
		return self
	end
	function META:Maxs(...)
		return CLIB.btDbvtAabbMm_Maxs(self.ptr, ...)
	end
	function META:FromCE(...)
		return CLIB.btDbvtAabbMm_FromCE(self.ptr, ...)
	end
	function META:Extents(...)
		return CLIB.btDbvtAabbMm_Extents(self.ptr, ...)
	end
	function META:Lengths(...)
		return CLIB.btDbvtAabbMm_Lengths(self.ptr, ...)
	end
	function META:Expand(...)
		return CLIB.btDbvtAabbMm_Expand(self.ptr, ...)
	end
	function META:FromPoints2(...)
		return CLIB.btDbvtAabbMm_FromPoints2(self.ptr, ...)
	end
	function META:Delete(...)
		return CLIB.btDbvtAabbMm_delete(self.ptr, ...)
	end
	function META:Mins(...)
		return CLIB.btDbvtAabbMm_Mins(self.ptr, ...)
	end
	function META:Contain(...)
		return CLIB.btDbvtAabbMm_Contain(self.ptr, ...)
	end
	function META:Classify(...)
		return CLIB.btDbvtAabbMm_Classify(self.ptr, ...)
	end
	function META:TMaxs(...)
		return CLIB.btDbvtAabbMm_tMaxs(self.ptr, ...)
	end
	function META:SignedExpand(...)
		return CLIB.btDbvtAabbMm_SignedExpand(self.ptr, ...)
	end
	function META:FromPoints(...)
		return CLIB.btDbvtAabbMm_FromPoints(self.ptr, ...)
	end
	function META:FromCR(...)
		return CLIB.btDbvtAabbMm_FromCR(self.ptr, ...)
	end
	function META:TMins(...)
		return CLIB.btDbvtAabbMm_tMins(self.ptr, ...)
	end
	function META:FromMM(...)
		return CLIB.btDbvtAabbMm_FromMM(self.ptr, ...)
	end
	function META:ProjectMinimum(...)
		return CLIB.btDbvtAabbMm_ProjectMinimum(self.ptr, ...)
	end
	function META:Center(...)
		return CLIB.btDbvtAabbMm_Center(self.ptr, ...)
	end
end
do -- btConeShapeZ
	local META = {}
	library.metatables.btConeShapeZ = META
	META.__index = function(s, k) return META[k] end
	function library.CreateConeShapeZ(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btConeShapeZ_new(...)
		return self
	end
end
do -- btConvexCast
	local META = {}
	library.metatables.btConvexCast = META
	META.__index = function(s, k) return META[k] end
	function META:Delete(...)
		return CLIB.btConvexCast_delete(self.ptr, ...)
	end
	function META:CalcTimeOfImpact(...)
		return CLIB.btConvexCast_calcTimeOfImpact(self.ptr, ...)
	end
end
do -- btAxisSweep3
	local META = {}
	library.metatables.btAxisSweep3 = META
	META.__index = function(s, k) return META[k] end
	function library.CreateAxisSweep3(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btAxisSweep3_new(...)
		return self
	end
	function META:SetOverlappingPairUserCallback(...)
		return CLIB.btAxisSweep3_setOverlappingPairUserCallback(self.ptr, ...)
	end
	function META:Quantize(...)
		return CLIB.btAxisSweep3_quantize(self.ptr, ...)
	end
	function META:UnQuantize(...)
		return CLIB.btAxisSweep3_unQuantize(self.ptr, ...)
	end
	function META:UpdateHandle(...)
		return CLIB.btAxisSweep3_updateHandle(self.ptr, ...)
	end
	function META:GetOverlappingPairUserCallback(...)
		return CLIB.btAxisSweep3_getOverlappingPairUserCallback(self.ptr, ...)
	end
	function META:GetHandle(...)
		return CLIB.btAxisSweep3_getHandle(self.ptr, ...)
	end
	function META:TestAabbOverlap(...)
		return CLIB.btAxisSweep3_testAabbOverlap(self.ptr, ...)
	end
	function META:AddHandle(...)
		return CLIB.btAxisSweep3_addHandle(self.ptr, ...)
	end
	function META:RemoveHandle(...)
		return CLIB.btAxisSweep3_removeHandle(self.ptr, ...)
	end
	function META:GetNumHandles(...)
		return CLIB.btAxisSweep3_getNumHandles(self.ptr, ...)
	end
end
do -- btAABB_plane
	local META = {}
	library.metatables.btAABB_plane = META
	META.__index = function(s, k) return META[k] end
	function META:Classify(...)
		return CLIB.btAABB_plane_classify(self.ptr, ...)
	end
end
do -- btRigidBody
	local META = {}
	library.metatables.btRigidBody = META
	META.__index = function(s, k) return META[k] end
	function library.CreateRigidBody(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btRigidBody_new(...)
		return self
	end
	function META:WantsSleeping(...)
		return CLIB.btRigidBody_wantsSleeping(self.ptr, ...)
	end
	function META:SetLinearVelocity(...)
		return CLIB.btRigidBody_setLinearVelocity(self.ptr, ...)
	end
	function META:GetCenterOfMassPosition(...)
		return CLIB.btRigidBody_getCenterOfMassPosition(self.ptr, ...)
	end
	function META:SetMotionState(...)
		return CLIB.btRigidBody_setMotionState(self.ptr, ...)
	end
	function META:SetDamping(...)
		return CLIB.btRigidBody_setDamping(self.ptr, ...)
	end
	function META:GetOrientation(...)
		return CLIB.btRigidBody_getOrientation(self.ptr, ...)
	end
	function META:GetVelocityInLocalPoint(...)
		return CLIB.btRigidBody_getVelocityInLocalPoint(self.ptr, ...)
	end
	function META:SetAngularFactor2(...)
		return CLIB.btRigidBody_setAngularFactor2(self.ptr, ...)
	end
	function META:SetAngularFactor(...)
		return CLIB.btRigidBody_setAngularFactor(self.ptr, ...)
	end
	function META:SetMassProps(...)
		return CLIB.btRigidBody_setMassProps(self.ptr, ...)
	end
	function META:SetAngularVelocity(...)
		return CLIB.btRigidBody_setAngularVelocity(self.ptr, ...)
	end
	function META:ApplyTorqueImpulse(...)
		return CLIB.btRigidBody_applyTorqueImpulse(self.ptr, ...)
	end
	function META:GetInvInertiaTensorWorld(...)
		return CLIB.btRigidBody_getInvInertiaTensorWorld(self.ptr, ...)
	end
	function META:GetGravity(...)
		return CLIB.btRigidBody_getGravity(self.ptr, ...)
	end
	function META:GetAngularFactor(...)
		return CLIB.btRigidBody_getAngularFactor(self.ptr, ...)
	end
	function META:GetFlags(...)
		return CLIB.btRigidBody_getFlags(self.ptr, ...)
	end
	function META:GetLinearDamping(...)
		return CLIB.btRigidBody_getLinearDamping(self.ptr, ...)
	end
	function META:Translate(...)
		return CLIB.btRigidBody_translate(self.ptr, ...)
	end
	function META:RemoveConstraintRef(...)
		return CLIB.btRigidBody_removeConstraintRef(self.ptr, ...)
	end
	function META:PredictIntegratedTransform(...)
		return CLIB.btRigidBody_predictIntegratedTransform(self.ptr, ...)
	end
	function META:ApplyCentralForce(...)
		return CLIB.btRigidBody_applyCentralForce(self.ptr, ...)
	end
	function META:ComputeAngularImpulseDenominator(...)
		return CLIB.btRigidBody_computeAngularImpulseDenominator(self.ptr, ...)
	end
	function META:SetGravity(...)
		return CLIB.btRigidBody_setGravity(self.ptr, ...)
	end
	function META:ApplyCentralImpulse(...)
		return CLIB.btRigidBody_applyCentralImpulse(self.ptr, ...)
	end
	function META:ComputeImpulseDenominator(...)
		return CLIB.btRigidBody_computeImpulseDenominator(self.ptr, ...)
	end
	function META:GetTotalTorque(...)
		return CLIB.btRigidBody_getTotalTorque(self.ptr, ...)
	end
	function META:GetLinearFactor(...)
		return CLIB.btRigidBody_getLinearFactor(self.ptr, ...)
	end
	function META:GetAabb(...)
		return CLIB.btRigidBody_getAabb(self.ptr, ...)
	end
	function META:GetAngularVelocity(...)
		return CLIB.btRigidBody_getAngularVelocity(self.ptr, ...)
	end
	function META:ClearForces(...)
		return CLIB.btRigidBody_clearForces(self.ptr, ...)
	end
	function META:GetFrictionSolverType(...)
		return CLIB.btRigidBody_getFrictionSolverType(self.ptr, ...)
	end
	function META:GetAngularDamping(...)
		return CLIB.btRigidBody_getAngularDamping(self.ptr, ...)
	end
	function META:SaveKinematicState(...)
		return CLIB.btRigidBody_saveKinematicState(self.ptr, ...)
	end
	function META:UpdateInertiaTensor(...)
		return CLIB.btRigidBody_updateInertiaTensor(self.ptr, ...)
	end
	function META:UpdateDeactivation(...)
		return CLIB.btRigidBody_updateDeactivation(self.ptr, ...)
	end
	function META:SetSleepingThresholds(...)
		return CLIB.btRigidBody_setSleepingThresholds(self.ptr, ...)
	end
	function META:SetLinearFactor(...)
		return CLIB.btRigidBody_setLinearFactor(self.ptr, ...)
	end
	function META:SetInvInertiaDiagLocal(...)
		return CLIB.btRigidBody_setInvInertiaDiagLocal(self.ptr, ...)
	end
	function META:SetFrictionSolverType(...)
		return CLIB.btRigidBody_setFrictionSolverType(self.ptr, ...)
	end
	function META:SetFlags(...)
		return CLIB.btRigidBody_setFlags(self.ptr, ...)
	end
	function META:SetContactSolverType(...)
		return CLIB.btRigidBody_setContactSolverType(self.ptr, ...)
	end
	function META:SetCenterOfMassTransform(...)
		return CLIB.btRigidBody_setCenterOfMassTransform(self.ptr, ...)
	end
	function META:IsInWorld(...)
		return CLIB.btRigidBody_isInWorld(self.ptr, ...)
	end
	function META:IntegrateVelocities(...)
		return CLIB.btRigidBody_integrateVelocities(self.ptr, ...)
	end
	function META:GetTotalForce(...)
		return CLIB.btRigidBody_getTotalForce(self.ptr, ...)
	end
	function META:GetInvMass(...)
		return CLIB.btRigidBody_getInvMass(self.ptr, ...)
	end
	function META:GetInvInertiaDiagLocal(...)
		return CLIB.btRigidBody_getInvInertiaDiagLocal(self.ptr, ...)
	end
	function META:GetConstraintRef(...)
		return CLIB.btRigidBody_getConstraintRef(self.ptr, ...)
	end
	function META:GetBroadphaseProxy(...)
		return CLIB.btRigidBody_getBroadphaseProxy(self.ptr, ...)
	end
	function META:GetAngularSleepingThreshold(...)
		return CLIB.btRigidBody_getAngularSleepingThreshold(self.ptr, ...)
	end
	function META:ApplyTorque(...)
		return CLIB.btRigidBody_applyTorque(self.ptr, ...)
	end
	function META:ApplyImpulse(...)
		return CLIB.btRigidBody_applyImpulse(self.ptr, ...)
	end
	function META:ApplyForce(...)
		return CLIB.btRigidBody_applyForce(self.ptr, ...)
	end
	function META:ApplyDamping(...)
		return CLIB.btRigidBody_applyDamping(self.ptr, ...)
	end
	function META:AddConstraintRef(...)
		return CLIB.btRigidBody_addConstraintRef(self.ptr, ...)
	end
	function META:GetContactSolverType(...)
		return CLIB.btRigidBody_getContactSolverType(self.ptr, ...)
	end
	function META:GetNumConstraintRefs(...)
		return CLIB.btRigidBody_getNumConstraintRefs(self.ptr, ...)
	end
	function META:GetMotionState(...)
		return CLIB.btRigidBody_getMotionState(self.ptr, ...)
	end
	function META:Upcast(...)
		return CLIB.btRigidBody_upcast(self.ptr, ...)
	end
	function META:GetLocalInertia(...)
		return CLIB.btRigidBody_getLocalInertia(self.ptr, ...)
	end
	function META:ProceedToTransform(...)
		return CLIB.btRigidBody_proceedToTransform(self.ptr, ...)
	end
	function META:GetLinearVelocity(...)
		return CLIB.btRigidBody_getLinearVelocity(self.ptr, ...)
	end
	function META:GetLinearSleepingThreshold(...)
		return CLIB.btRigidBody_getLinearSleepingThreshold(self.ptr, ...)
	end
	function META:ApplyGravity(...)
		return CLIB.btRigidBody_applyGravity(self.ptr, ...)
	end
	function META:SetNewBroadphaseProxy(...)
		return CLIB.btRigidBody_setNewBroadphaseProxy(self.ptr, ...)
	end
	function META:ComputeGyroscopicForceExplicit(...)
		return CLIB.btRigidBody_computeGyroscopicForceExplicit(self.ptr, ...)
	end
	function META:GetCenterOfMassTransform(...)
		return CLIB.btRigidBody_getCenterOfMassTransform(self.ptr, ...)
	end
end
do -- btAABB_find
	local META = {}
	library.metatables.btAABB_find = META
	META.__index = function(s, k) return META[k] end
	function META:Intersection(...)
		return CLIB.btAABB_find_intersection(self.ptr, ...)
	end
end
do -- btSparseSdf
	local META = {}
	library.metatables.btSparseSdf = META
	META.__index = function(s, k) return META[k] end
	function library.CreateSparseSdf(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btSparseSdf_new(...)
		return self
	end
	function META:Delete(...)
		return CLIB.btSparseSdf_delete(self.ptr, ...)
	end
end
do -- btShapeHull
	local META = {}
	library.metatables.btShapeHull = META
	META.__index = function(s, k) return META[k] end
	function library.CreateShapeHull(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btShapeHull_new(...)
		return self
	end
	function META:BuildHull(...)
		return CLIB.btShapeHull_buildHull(self.ptr, ...)
	end
	function META:Delete(...)
		return CLIB.btShapeHull_delete(self.ptr, ...)
	end
	function META:NumIndices(...)
		return CLIB.btShapeHull_numIndices(self.ptr, ...)
	end
	function META:GetVertexPointer(...)
		return CLIB.btShapeHull_getVertexPointer(self.ptr, ...)
	end
	function META:NumVertices(...)
		return CLIB.btShapeHull_numVertices(self.ptr, ...)
	end
	function META:GetIndexPointer(...)
		return CLIB.btShapeHull_getIndexPointer(self.ptr, ...)
	end
	function META:NumTriangles(...)
		return CLIB.btShapeHull_numTriangles(self.ptr, ...)
	end
end
do -- btUnionFind
	local META = {}
	library.metatables.btUnionFind = META
	META.__index = function(s, k) return META[k] end
	function library.CreateUnionFind(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btUnionFind_new(...)
		return self
	end
	function META:Delete(...)
		return CLIB.btUnionFind_delete(self.ptr, ...)
	end
	function META:Free(...)
		return CLIB.btUnionFind_Free(self.ptr, ...)
	end
	function META:Unite(...)
		return CLIB.btUnionFind_unite(self.ptr, ...)
	end
	function META:SortIslands(...)
		return CLIB.btUnionFind_sortIslands(self.ptr, ...)
	end
	function META:Reset(...)
		return CLIB.btUnionFind_reset(self.ptr, ...)
	end
	function META:GetNumElements(...)
		return CLIB.btUnionFind_getNumElements(self.ptr, ...)
	end
	function META:Find(...)
		return CLIB.btUnionFind_find(self.ptr, ...)
	end
	function META:GetElement(...)
		return CLIB.btUnionFind_getElement(self.ptr, ...)
	end
	function META:Find2(...)
		return CLIB.btUnionFind_find2(self.ptr, ...)
	end
	function META:IsRoot(...)
		return CLIB.btUnionFind_isRoot(self.ptr, ...)
	end
	function META:Allocate(...)
		return CLIB.btUnionFind_allocate(self.ptr, ...)
	end
end
do -- btWheelInfo
	local META = {}
	library.metatables.btWheelInfo = META
	META.__index = function(s, k) return META[k] end
	function library.CreateWheelInfo(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btWheelInfo_new(...)
		return self
	end
	function META:GetWheelAxleCS(...)
		return CLIB.btWheelInfo_getWheelAxleCS(self.ptr, ...)
	end
	function META:Delete(...)
		return CLIB.btWheelInfo_delete(self.ptr, ...)
	end
	function META:GetWorldTransform(...)
		return CLIB.btWheelInfo_getWorldTransform(self.ptr, ...)
	end
	function META:SetWheelsSuspensionForce(...)
		return CLIB.btWheelInfo_setWheelsSuspensionForce(self.ptr, ...)
	end
	function META:SetWheelsDampingRelaxation(...)
		return CLIB.btWheelInfo_setWheelsDampingRelaxation(self.ptr, ...)
	end
	function META:SetWheelsDampingCompression(...)
		return CLIB.btWheelInfo_setWheelsDampingCompression(self.ptr, ...)
	end
	function META:SetWheelDirectionCS(...)
		return CLIB.btWheelInfo_setWheelDirectionCS(self.ptr, ...)
	end
	function META:SetSuspensionStiffness(...)
		return CLIB.btWheelInfo_setSuspensionStiffness(self.ptr, ...)
	end
	function META:SetSuspensionRestLength1(...)
		return CLIB.btWheelInfo_setSuspensionRestLength1(self.ptr, ...)
	end
	function META:SetSuspensionRelativeVelocity(...)
		return CLIB.btWheelInfo_setSuspensionRelativeVelocity(self.ptr, ...)
	end
	function META:SetSteering(...)
		return CLIB.btWheelInfo_setSteering(self.ptr, ...)
	end
	function META:SetMaxSuspensionTravelCm(...)
		return CLIB.btWheelInfo_setMaxSuspensionTravelCm(self.ptr, ...)
	end
	function META:SetMaxSuspensionForce(...)
		return CLIB.btWheelInfo_setMaxSuspensionForce(self.ptr, ...)
	end
	function META:SetFrictionSlip(...)
		return CLIB.btWheelInfo_setFrictionSlip(self.ptr, ...)
	end
	function META:SetDeltaRotation(...)
		return CLIB.btWheelInfo_setDeltaRotation(self.ptr, ...)
	end
	function META:SetClippedInvContactDotSuspension(...)
		return CLIB.btWheelInfo_setClippedInvContactDotSuspension(self.ptr, ...)
	end
	function META:SetClientInfo(...)
		return CLIB.btWheelInfo_setClientInfo(self.ptr, ...)
	end
	function META:SetChassisConnectionPointCS(...)
		return CLIB.btWheelInfo_setChassisConnectionPointCS(self.ptr, ...)
	end
	function META:GetSteering(...)
		return CLIB.btWheelInfo_getSteering(self.ptr, ...)
	end
	function META:SetBrake(...)
		return CLIB.btWheelInfo_setBrake(self.ptr, ...)
	end
	function META:SetBIsFrontWheel(...)
		return CLIB.btWheelInfo_setBIsFrontWheel(self.ptr, ...)
	end
	function META:SetWorldTransform(...)
		return CLIB.btWheelInfo_setWorldTransform(self.ptr, ...)
	end
	function META:GetWheelsRadius(...)
		return CLIB.btWheelInfo_getWheelsRadius(self.ptr, ...)
	end
	function META:GetSuspensionRelativeVelocity(...)
		return CLIB.btWheelInfo_getSuspensionRelativeVelocity(self.ptr, ...)
	end
	function META:GetSkidInfo(...)
		return CLIB.btWheelInfo_getSkidInfo(self.ptr, ...)
	end
	function META:GetRotation(...)
		return CLIB.btWheelInfo_getRotation(self.ptr, ...)
	end
	function META:GetRollInfluence(...)
		return CLIB.btWheelInfo_getRollInfluence(self.ptr, ...)
	end
	function META:GetRaycastInfo(...)
		return CLIB.btWheelInfo_getRaycastInfo(self.ptr, ...)
	end
	function META:GetMaxSuspensionTravelCm(...)
		return CLIB.btWheelInfo_getMaxSuspensionTravelCm(self.ptr, ...)
	end
	function META:GetFrictionSlip(...)
		return CLIB.btWheelInfo_getFrictionSlip(self.ptr, ...)
	end
	function META:GetChassisConnectionPointCS(...)
		return CLIB.btWheelInfo_getChassisConnectionPointCS(self.ptr, ...)
	end
	function META:GetBIsFrontWheel(...)
		return CLIB.btWheelInfo_getBIsFrontWheel(self.ptr, ...)
	end
	function META:SetRotation(...)
		return CLIB.btWheelInfo_setRotation(self.ptr, ...)
	end
	function META:GetWheelDirectionCS(...)
		return CLIB.btWheelInfo_getWheelDirectionCS(self.ptr, ...)
	end
	function META:GetSuspensionStiffness(...)
		return CLIB.btWheelInfo_getSuspensionStiffness(self.ptr, ...)
	end
	function META:GetSuspensionRestLength(...)
		return CLIB.btWheelInfo_getSuspensionRestLength(self.ptr, ...)
	end
	function META:UpdateWheel(...)
		return CLIB.btWheelInfo_updateWheel(self.ptr, ...)
	end
	function META:GetWheelsDampingCompression(...)
		return CLIB.btWheelInfo_getWheelsDampingCompression(self.ptr, ...)
	end
	function META:GetEngineForce(...)
		return CLIB.btWheelInfo_getEngineForce(self.ptr, ...)
	end
	function META:GetDeltaRotation(...)
		return CLIB.btWheelInfo_getDeltaRotation(self.ptr, ...)
	end
	function META:SetEngineForce(...)
		return CLIB.btWheelInfo_setEngineForce(self.ptr, ...)
	end
	function META:SetRollInfluence(...)
		return CLIB.btWheelInfo_setRollInfluence(self.ptr, ...)
	end
	function META:SetWheelAxleCS(...)
		return CLIB.btWheelInfo_setWheelAxleCS(self.ptr, ...)
	end
	function META:GetMaxSuspensionForce(...)
		return CLIB.btWheelInfo_getMaxSuspensionForce(self.ptr, ...)
	end
	function META:GetClippedInvContactDotSuspension(...)
		return CLIB.btWheelInfo_getClippedInvContactDotSuspension(self.ptr, ...)
	end
	function META:GetBrake(...)
		return CLIB.btWheelInfo_getBrake(self.ptr, ...)
	end
	function META:GetWheelsDampingRelaxation(...)
		return CLIB.btWheelInfo_getWheelsDampingRelaxation(self.ptr, ...)
	end
	function META:GetClientInfo(...)
		return CLIB.btWheelInfo_getClientInfo(self.ptr, ...)
	end
	function META:SetWheelsRadius(...)
		return CLIB.btWheelInfo_setWheelsRadius(self.ptr, ...)
	end
	function META:SetSkidInfo(...)
		return CLIB.btWheelInfo_setSkidInfo(self.ptr, ...)
	end
	function META:GetSuspensionRestLength1(...)
		return CLIB.btWheelInfo_getSuspensionRestLength1(self.ptr, ...)
	end
	function META:GetWheelsSuspensionForce(...)
		return CLIB.btWheelInfo_getWheelsSuspensionForce(self.ptr, ...)
	end
end
do -- btConeShape
	local META = {}
	library.metatables.btConeShape = META
	META.__index = function(s, k) return META[k] end
	function library.CreateConeShape(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btConeShape_new(...)
		return self
	end
	function META:GetHeight(...)
		return CLIB.btConeShape_getHeight(self.ptr, ...)
	end
	function META:SetRadius(...)
		return CLIB.btConeShape_setRadius(self.ptr, ...)
	end
	function META:SetConeUpIndex(...)
		return CLIB.btConeShape_setConeUpIndex(self.ptr, ...)
	end
	function META:GetConeUpIndex(...)
		return CLIB.btConeShape_getConeUpIndex(self.ptr, ...)
	end
	function META:GetRadius(...)
		return CLIB.btConeShape_getRadius(self.ptr, ...)
	end
	function META:SetHeight(...)
		return CLIB.btConeShape_setHeight(self.ptr, ...)
	end
end
do -- btAABB_appy
	local META = {}
	library.metatables.btAABB_appy = META
	META.__index = function(s, k) return META[k] end
	function META:Transform(...)
		return CLIB.btAABB_appy_transform(self.ptr, ...)
	end
end
do -- btDbvtProxy
	local META = {}
	library.metatables.btDbvtProxy = META
	META.__index = function(s, k) return META[k] end
	function META:SetLeaf(...)
		return CLIB.btDbvtProxy_setLeaf(self.ptr, ...)
	end
	function META:GetLinks(...)
		return CLIB.btDbvtProxy_getLinks(self.ptr, ...)
	end
	function META:SetStage(...)
		return CLIB.btDbvtProxy_setStage(self.ptr, ...)
	end
	function META:GetStage(...)
		return CLIB.btDbvtProxy_getStage(self.ptr, ...)
	end
	function META:GetLeaf(...)
		return CLIB.btDbvtProxy_getLeaf(self.ptr, ...)
	end
end
do -- btMultiBody
	local META = {}
	library.metatables.btMultiBody = META
	META.__index = function(s, k) return META[k] end
	function library.CreateMultiBody(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btMultiBody_new(...)
		return self
	end
	function META:SetBaseInertia(...)
		return CLIB.btMultiBody_setBaseInertia(self.ptr, ...)
	end
	function META:IsPosUpdated(...)
		return CLIB.btMultiBody_isPosUpdated(self.ptr, ...)
	end
	function META:GetCanSleep(...)
		return CLIB.btMultiBody_getCanSleep(self.ptr, ...)
	end
	function META:WakeUp(...)
		return CLIB.btMultiBody_wakeUp(self.ptr, ...)
	end
	function META:GetBaseCollider(...)
		return CLIB.btMultiBody_getBaseCollider(self.ptr, ...)
	end
	function META:SetupSpherical(...)
		return CLIB.btMultiBody_setupSpherical(self.ptr, ...)
	end
	function META:GetUseGyroTerm(...)
		return CLIB.btMultiBody_getUseGyroTerm(self.ptr, ...)
	end
	function META:GetJointTorqueMultiDof(...)
		return CLIB.btMultiBody_getJointTorqueMultiDof(self.ptr, ...)
	end
	function META:SetHasSelfCollision(...)
		return CLIB.btMultiBody_setHasSelfCollision(self.ptr, ...)
	end
	function META:SetupPrismatic(...)
		return CLIB.btMultiBody_setupPrismatic(self.ptr, ...)
	end
	function META:AddBaseConstraintForce(...)
		return CLIB.btMultiBody_addBaseConstraintForce(self.ptr, ...)
	end
	function META:SetCompanionId(...)
		return CLIB.btMultiBody_setCompanionId(self.ptr, ...)
	end
	function META:GetLinearDamping(...)
		return CLIB.btMultiBody_getLinearDamping(self.ptr, ...)
	end
	function META:ApplyDeltaVeeMultiDof(...)
		return CLIB.btMultiBody_applyDeltaVeeMultiDof(self.ptr, ...)
	end
	function META:StepPositionsMultiDof(...)
		return CLIB.btMultiBody_stepPositionsMultiDof(self.ptr, ...)
	end
	function META:SetBaseWorldTransform(...)
		return CLIB.btMultiBody_setBaseWorldTransform(self.ptr, ...)
	end
	function META:ClearConstraintForces(...)
		return CLIB.btMultiBody_clearConstraintForces(self.ptr, ...)
	end
	function META:SetJointPos(...)
		return CLIB.btMultiBody_setJointPos(self.ptr, ...)
	end
	function META:ProcessDeltaVeeMultiDof2(...)
		return CLIB.btMultiBody_processDeltaVeeMultiDof2(self.ptr, ...)
	end
	function META:SetUserIndex(...)
		return CLIB.btMultiBody_setUserIndex(self.ptr, ...)
	end
	function META:GetBasePos(...)
		return CLIB.btMultiBody_getBasePos(self.ptr, ...)
	end
	function META:UseGlobalVelocities(...)
		return CLIB.btMultiBody_useGlobalVelocities(self.ptr, ...)
	end
	function META:GetLink(...)
		return CLIB.btMultiBody_getLink(self.ptr, ...)
	end
	function META:SetupFixed(...)
		return CLIB.btMultiBody_setupFixed(self.ptr, ...)
	end
	function META:HasFixedBase(...)
		return CLIB.btMultiBody_hasFixedBase(self.ptr, ...)
	end
	function META:SetMaxCoordinateVelocity(...)
		return CLIB.btMultiBody_setMaxCoordinateVelocity(self.ptr, ...)
	end
	function META:StepVelocitiesMultiDof(...)
		return CLIB.btMultiBody_stepVelocitiesMultiDof(self.ptr, ...)
	end
	function META:GetLinkTorque(...)
		return CLIB.btMultiBody_getLinkTorque(self.ptr, ...)
	end
	function META:GetCompanionId(...)
		return CLIB.btMultiBody_getCompanionId(self.ptr, ...)
	end
	function META:GetBaseName(...)
		return CLIB.btMultiBody_getBaseName(self.ptr, ...)
	end
	function META:GetLinkInertia(...)
		return CLIB.btMultiBody_getLinkInertia(self.ptr, ...)
	end
	function META:AddJointTorqueMultiDof2(...)
		return CLIB.btMultiBody_addJointTorqueMultiDof2(self.ptr, ...)
	end
	function META:SetBaseVel(...)
		return CLIB.btMultiBody_setBaseVel(self.ptr, ...)
	end
	function META:SetBasePos(...)
		return CLIB.btMultiBody_setBasePos(self.ptr, ...)
	end
	function META:GetVelocityVector(...)
		return CLIB.btMultiBody_getVelocityVector(self.ptr, ...)
	end
	function META:WorldPosToLocal(...)
		return CLIB.btMultiBody_worldPosToLocal(self.ptr, ...)
	end
	function META:SetUseGyroTerm(...)
		return CLIB.btMultiBody_setUseGyroTerm(self.ptr, ...)
	end
	function META:GetUserIndex2(...)
		return CLIB.btMultiBody_getUserIndex2(self.ptr, ...)
	end
	function META:SetNumLinks(...)
		return CLIB.btMultiBody_setNumLinks(self.ptr, ...)
	end
	function META:GetRVector(...)
		return CLIB.btMultiBody_getRVector(self.ptr, ...)
	end
	function META:GetBaseInertia(...)
		return CLIB.btMultiBody_getBaseInertia(self.ptr, ...)
	end
	function META:GetJointPosMultiDof(...)
		return CLIB.btMultiBody_getJointPosMultiDof(self.ptr, ...)
	end
	function META:GetLinkForce(...)
		return CLIB.btMultiBody_getLinkForce(self.ptr, ...)
	end
	function META:Delete(...)
		return CLIB.btMultiBody_delete(self.ptr, ...)
	end
	function META:GetParent(...)
		return CLIB.btMultiBody_getParent(self.ptr, ...)
	end
	function META:GetBaseVel(...)
		return CLIB.btMultiBody_getBaseVel(self.ptr, ...)
	end
	function META:GetBaseOmega(...)
		return CLIB.btMultiBody_getBaseOmega(self.ptr, ...)
	end
	function META:AddLinkTorque(...)
		return CLIB.btMultiBody_addLinkTorque(self.ptr, ...)
	end
	function META:AddJointTorqueMultiDof(...)
		return CLIB.btMultiBody_addJointTorqueMultiDof(self.ptr, ...)
	end
	function META:UpdateCollisionObjectWorldTransforms(...)
		return CLIB.btMultiBody_updateCollisionObjectWorldTransforms(self.ptr, ...)
	end
	function META:GetJointVel(...)
		return CLIB.btMultiBody_getJointVel(self.ptr, ...)
	end
	function META:LocalPosToWorld(...)
		return CLIB.btMultiBody_localPosToWorld(self.ptr, ...)
	end
	function META:UseRK4Integration(...)
		return CLIB.btMultiBody_useRK4Integration(self.ptr, ...)
	end
	function META:AddLinkConstraintForce(...)
		return CLIB.btMultiBody_addLinkConstraintForce(self.ptr, ...)
	end
	function META:GetMaxCoordinateVelocity(...)
		return CLIB.btMultiBody_getMaxCoordinateVelocity(self.ptr, ...)
	end
	function META:FillConstraintJacobianMultiDof(...)
		return CLIB.btMultiBody_fillConstraintJacobianMultiDof(self.ptr, ...)
	end
	function META:SetCanSleep(...)
		return CLIB.btMultiBody_setCanSleep(self.ptr, ...)
	end
	function META:SetBaseCollider(...)
		return CLIB.btMultiBody_setBaseCollider(self.ptr, ...)
	end
	function META:CheckMotionAndSleepIfRequired(...)
		return CLIB.btMultiBody_checkMotionAndSleepIfRequired(self.ptr, ...)
	end
	function META:GetKineticEnergy(...)
		return CLIB.btMultiBody_getKineticEnergy(self.ptr, ...)
	end
	function META:GetUserIndex(...)
		return CLIB.btMultiBody_getUserIndex(self.ptr, ...)
	end
	function META:FillContactJacobianMultiDof(...)
		return CLIB.btMultiBody_fillContactJacobianMultiDof(self.ptr, ...)
	end
	function META:AddBaseConstraintTorque(...)
		return CLIB.btMultiBody_addBaseConstraintTorque(self.ptr, ...)
	end
	function META:ClearForcesAndTorques(...)
		return CLIB.btMultiBody_clearForcesAndTorques(self.ptr, ...)
	end
	function META:GetWorldToBaseRot(...)
		return CLIB.btMultiBody_getWorldToBaseRot(self.ptr, ...)
	end
	function META:SetBaseOmega(...)
		return CLIB.btMultiBody_setBaseOmega(self.ptr, ...)
	end
	function META:GetJointTorque(...)
		return CLIB.btMultiBody_getJointTorque(self.ptr, ...)
	end
	function META:SetBaseMass(...)
		return CLIB.btMultiBody_setBaseMass(self.ptr, ...)
	end
	function META:SetUserIndex2(...)
		return CLIB.btMultiBody_setUserIndex2(self.ptr, ...)
	end
	function META:HasSelfCollision(...)
		return CLIB.btMultiBody_hasSelfCollision(self.ptr, ...)
	end
	function META:LocalFrameToWorld(...)
		return CLIB.btMultiBody_localFrameToWorld(self.ptr, ...)
	end
	function META:SetJointVelMultiDof(...)
		return CLIB.btMultiBody_setJointVelMultiDof(self.ptr, ...)
	end
	function META:SetJointPosMultiDof(...)
		return CLIB.btMultiBody_setJointPosMultiDof(self.ptr, ...)
	end
	function META:CalculateSerializeBufferSize(...)
		return CLIB.btMultiBody_calculateSerializeBufferSize(self.ptr, ...)
	end
	function META:ComputeAccelerationsArticulatedBodyAlgorithmMultiDof(...)
		return CLIB.btMultiBody_computeAccelerationsArticulatedBodyAlgorithmMultiDof(self.ptr, ...)
	end
	function META:IsAwake(...)
		return CLIB.btMultiBody_isAwake(self.ptr, ...)
	end
	function META:FinalizeMultiDof(...)
		return CLIB.btMultiBody_finalizeMultiDof(self.ptr, ...)
	end
	function META:SetupRevolute(...)
		return CLIB.btMultiBody_setupRevolute(self.ptr, ...)
	end
	function META:GetAngularDamping(...)
		return CLIB.btMultiBody_getAngularDamping(self.ptr, ...)
	end
	function META:SetWorldToBaseRot(...)
		return CLIB.btMultiBody_setWorldToBaseRot(self.ptr, ...)
	end
	function META:SetPosUpdated(...)
		return CLIB.btMultiBody_setPosUpdated(self.ptr, ...)
	end
	function META:AddLinkConstraintTorque(...)
		return CLIB.btMultiBody_addLinkConstraintTorque(self.ptr, ...)
	end
	function META:AddBaseForce(...)
		return CLIB.btMultiBody_addBaseForce(self.ptr, ...)
	end
	function META:GetNumPosVars(...)
		return CLIB.btMultiBody_getNumPosVars(self.ptr, ...)
	end
	function META:GetBaseMass(...)
		return CLIB.btMultiBody_getBaseMass(self.ptr, ...)
	end
	function META:IsUsingGlobalVelocities(...)
		return CLIB.btMultiBody_isUsingGlobalVelocities(self.ptr, ...)
	end
	function META:CalcAccelerationDeltasMultiDof(...)
		return CLIB.btMultiBody_calcAccelerationDeltasMultiDof(self.ptr, ...)
	end
	function META:InternalNeedsJointFeedback(...)
		return CLIB.btMultiBody_internalNeedsJointFeedback(self.ptr, ...)
	end
	function META:SetUserPointer(...)
		return CLIB.btMultiBody_setUserPointer(self.ptr, ...)
	end
	function META:GetNumDofs(...)
		return CLIB.btMultiBody_getNumDofs(self.ptr, ...)
	end
	function META:SetMaxAppliedImpulse(...)
		return CLIB.btMultiBody_setMaxAppliedImpulse(self.ptr, ...)
	end
	function META:Serialize(...)
		return CLIB.btMultiBody_serialize(self.ptr, ...)
	end
	function META:SetupPlanar(...)
		return CLIB.btMultiBody_setupPlanar(self.ptr, ...)
	end
	function META:GetUserPointer(...)
		return CLIB.btMultiBody_getUserPointer(self.ptr, ...)
	end
	function META:SetBaseName(...)
		return CLIB.btMultiBody_setBaseName(self.ptr, ...)
	end
	function META:GetBaseForce(...)
		return CLIB.btMultiBody_getBaseForce(self.ptr, ...)
	end
	function META:GetMaxAppliedImpulse(...)
		return CLIB.btMultiBody_getMaxAppliedImpulse(self.ptr, ...)
	end
	function META:IsUsingRK4Integration(...)
		return CLIB.btMultiBody_isUsingRK4Integration(self.ptr, ...)
	end
	function META:WorldDirToLocal(...)
		return CLIB.btMultiBody_worldDirToLocal(self.ptr, ...)
	end
	function META:SetLinearDamping(...)
		return CLIB.btMultiBody_setLinearDamping(self.ptr, ...)
	end
	function META:SetAngularDamping(...)
		return CLIB.btMultiBody_setAngularDamping(self.ptr, ...)
	end
	function META:ForwardKinematics(...)
		return CLIB.btMultiBody_forwardKinematics(self.ptr, ...)
	end
	function META:LocalDirToWorld(...)
		return CLIB.btMultiBody_localDirToWorld(self.ptr, ...)
	end
	function META:AddJointTorque(...)
		return CLIB.btMultiBody_addJointTorque(self.ptr, ...)
	end
	function META:SetJointVel(...)
		return CLIB.btMultiBody_setJointVel(self.ptr, ...)
	end
	function META:GetParentToLocalRot(...)
		return CLIB.btMultiBody_getParentToLocalRot(self.ptr, ...)
	end
	function META:ApplyDeltaVeeMultiDof2(...)
		return CLIB.btMultiBody_applyDeltaVeeMultiDof2(self.ptr, ...)
	end
	function META:ClearVelocities(...)
		return CLIB.btMultiBody_clearVelocities(self.ptr, ...)
	end
	function META:AddBaseTorque(...)
		return CLIB.btMultiBody_addBaseTorque(self.ptr, ...)
	end
	function META:GoToSleep(...)
		return CLIB.btMultiBody_goToSleep(self.ptr, ...)
	end
	function META:AddLinkForce(...)
		return CLIB.btMultiBody_addLinkForce(self.ptr, ...)
	end
	function META:GetAngularMomentum(...)
		return CLIB.btMultiBody_getAngularMomentum(self.ptr, ...)
	end
	function META:GetBaseTorque(...)
		return CLIB.btMultiBody_getBaseTorque(self.ptr, ...)
	end
	function META:GetBaseWorldTransform(...)
		return CLIB.btMultiBody_getBaseWorldTransform(self.ptr, ...)
	end
	function META:GetJointPos(...)
		return CLIB.btMultiBody_getJointPos(self.ptr, ...)
	end
	function META:GetJointVelMultiDof(...)
		return CLIB.btMultiBody_getJointVelMultiDof(self.ptr, ...)
	end
	function META:GetLinkMass(...)
		return CLIB.btMultiBody_getLinkMass(self.ptr, ...)
	end
	function META:GetNumLinks(...)
		return CLIB.btMultiBody_getNumLinks(self.ptr, ...)
	end
end
do -- btAABB_has
	local META = {}
	library.metatables.btAABB_has = META
	META.__index = function(s, k) return META[k] end
	function META:Collision(...)
		return CLIB.btAABB_has_collision(self.ptr, ...)
	end
end
do -- btTriangle
	local META = {}
	library.metatables.btTriangle = META
	META.__index = function(s, k) return META[k] end
	function library.CreateTriangle(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btTriangle_new(...)
		return self
	end
	function META:GetTriangleIndex(...)
		return CLIB.btTriangle_getTriangleIndex(self.ptr, ...)
	end
	function META:GetVertex2(...)
		return CLIB.btTriangle_getVertex2(self.ptr, ...)
	end
	function META:Delete(...)
		return CLIB.btTriangle_delete(self.ptr, ...)
	end
	function META:SetVertex2(...)
		return CLIB.btTriangle_setVertex2(self.ptr, ...)
	end
	function META:SetVertex1(...)
		return CLIB.btTriangle_setVertex1(self.ptr, ...)
	end
	function META:SetVertex0(...)
		return CLIB.btTriangle_setVertex0(self.ptr, ...)
	end
	function META:SetTriangleIndex(...)
		return CLIB.btTriangle_setTriangleIndex(self.ptr, ...)
	end
	function META:SetPartId(...)
		return CLIB.btTriangle_setPartId(self.ptr, ...)
	end
	function META:GetPartId(...)
		return CLIB.btTriangle_getPartId(self.ptr, ...)
	end
	function META:GetVertex1(...)
		return CLIB.btTriangle_getVertex1(self.ptr, ...)
	end
	function META:GetVertex0(...)
		return CLIB.btTriangle_getVertex0(self.ptr, ...)
	end
end
do -- btBoxShape
	local META = {}
	library.metatables.btBoxShape = META
	function META:__index(k)
		local v

		v = META[k]
		if v ~= nil then
			return v
		end
		v = library.metatables.btConvexInternalShape.__index(self, k)
		if v ~= nil then
			return v
		end
	end
	function library.CreateBoxShape(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btBoxShape_new(...)
		return self
	end
	function library.CreateBoxShape2(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btBoxShape_new3(...)
		return self
	end
	function library.CreateBoxShape3(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btBoxShape_new2(...)
		return self
	end
	function META:GetHalfExtentsWithoutMargin(...)
		return CLIB.btBoxShape_getHalfExtentsWithoutMargin(self.ptr, ...)
	end
	function META:GetPlaneEquation(...)
		return CLIB.btBoxShape_getPlaneEquation(self.ptr, ...)
	end
	function META:GetHalfExtentsWithMargin(...)
		return CLIB.btBoxShape_getHalfExtentsWithMargin(self.ptr, ...)
	end
end
do -- btSoftBody
	local META = {}
	library.metatables.btSoftBody = META
	META.__index = function(s, k) return META[k] end
	function library.CreateSoftBody(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btSoftBody_new(...)
		return self
	end
	function library.CreateSoftBody2(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btSoftBody_new2(...)
		return self
	end
	function META:SetTimeacc(...)
		return CLIB.btSoftBody_setTimeacc(self.ptr, ...)
	end
	function META:GetNotes(...)
		return CLIB.btSoftBody_getNotes(self.ptr, ...)
	end
	function META:GetTetraVertexNormalData2(...)
		return CLIB.btSoftBody_getTetraVertexNormalData2(self.ptr, ...)
	end
	function META:GetTotalMass(...)
		return CLIB.btSoftBody_getTotalMass(self.ptr, ...)
	end
	function META:GetNodes(...)
		return CLIB.btSoftBody_getNodes(self.ptr, ...)
	end
	function META:GetAabb(...)
		return CLIB.btSoftBody_getAabb(self.ptr, ...)
	end
	function META:GetWorldInfo(...)
		return CLIB.btSoftBody_getWorldInfo(self.ptr, ...)
	end
	function META:RandomizeConstraints(...)
		return CLIB.btSoftBody_randomizeConstraints(self.ptr, ...)
	end
	function META:GetLinkVertexData(...)
		return CLIB.btSoftBody_getLinkVertexData(self.ptr, ...)
	end
	function META:AppendNote(...)
		return CLIB.btSoftBody_appendNote(self.ptr, ...)
	end
	function META:AppendAngularJoint3(...)
		return CLIB.btSoftBody_appendAngularJoint3(self.ptr, ...)
	end
	function META:GetFaceVertexNormalData2(...)
		return CLIB.btSoftBody_getFaceVertexNormalData2(self.ptr, ...)
	end
	function META:GetLinks(...)
		return CLIB.btSoftBody_getLinks(self.ptr, ...)
	end
	function META:GetFaceVertexData(...)
		return CLIB.btSoftBody_getFaceVertexData(self.ptr, ...)
	end
	function META:SolveClusters(...)
		return CLIB.btSoftBody_solveClusters(self.ptr, ...)
	end
	function META:GetClusters(...)
		return CLIB.btSoftBody_getClusters(self.ptr, ...)
	end
	function META:AppendFace2(...)
		return CLIB.btSoftBody_appendFace2(self.ptr, ...)
	end
	function META:SetSoftBodySolver(...)
		return CLIB.btSoftBody_setSoftBodySolver(self.ptr, ...)
	end
	function META:AppendLinearJoint3(...)
		return CLIB.btSoftBody_appendLinearJoint3(self.ptr, ...)
	end
	function META:SetVolumeDensity(...)
		return CLIB.btSoftBody_setVolumeDensity(self.ptr, ...)
	end
	function META:AppendAngularJoint2(...)
		return CLIB.btSoftBody_appendAngularJoint2(self.ptr, ...)
	end
	function META:CutLink(...)
		return CLIB.btSoftBody_cutLink(self.ptr, ...)
	end
	function META:DefaultCollisionHandler2(...)
		return CLIB.btSoftBody_defaultCollisionHandler2(self.ptr, ...)
	end
	function META:RayTest2(...)
		return CLIB.btSoftBody_rayTest2(self.ptr, ...)
	end
	function META:ClusterCom(...)
		return CLIB.btSoftBody_clusterCom(self.ptr, ...)
	end
	function META:UpdateClusters(...)
		return CLIB.btSoftBody_updateClusters(self.ptr, ...)
	end
	function META:AddVelocity(...)
		return CLIB.btSoftBody_addVelocity(self.ptr, ...)
	end
	function META:SetTotalDensity(...)
		return CLIB.btSoftBody_setTotalDensity(self.ptr, ...)
	end
	function META:ClusterCount(...)
		return CLIB.btSoftBody_clusterCount(self.ptr, ...)
	end
	function META:GetVolume(...)
		return CLIB.btSoftBody_getVolume(self.ptr, ...)
	end
	function META:CheckLink(...)
		return CLIB.btSoftBody_checkLink(self.ptr, ...)
	end
	function META:GetTag(...)
		return CLIB.btSoftBody_getTag(self.ptr, ...)
	end
	function META:ApplyClusters(...)
		return CLIB.btSoftBody_applyClusters(self.ptr, ...)
	end
	function META:GetTetras(...)
		return CLIB.btSoftBody_getTetras(self.ptr, ...)
	end
	function META:AddVelocity2(...)
		return CLIB.btSoftBody_addVelocity2(self.ptr, ...)
	end
	function META:AppendNote2(...)
		return CLIB.btSoftBody_appendNote2(self.ptr, ...)
	end
	function META:GenerateBendingConstraints(...)
		return CLIB.btSoftBody_generateBendingConstraints(self.ptr, ...)
	end
	function META:AppendTetra(...)
		return CLIB.btSoftBody_appendTetra(self.ptr, ...)
	end
	function META:InitializeClusters(...)
		return CLIB.btSoftBody_initializeClusters(self.ptr, ...)
	end
	function META:GetTetraVertexNormalData(...)
		return CLIB.btSoftBody_getTetraVertexNormalData(self.ptr, ...)
	end
	function META:UpdateNormals(...)
		return CLIB.btSoftBody_updateNormals(self.ptr, ...)
	end
	function META:UpdateBounds(...)
		return CLIB.btSoftBody_updateBounds(self.ptr, ...)
	end
	function META:Translate(...)
		return CLIB.btSoftBody_translate(self.ptr, ...)
	end
	function META:Transform(...)
		return CLIB.btSoftBody_transform(self.ptr, ...)
	end
	function META:SolveConstraints(...)
		return CLIB.btSoftBody_solveConstraints(self.ptr, ...)
	end
	function META:SolveCommonConstraints(...)
		return CLIB.btSoftBody_solveCommonConstraints(self.ptr, ...)
	end
	function META:SolveClusters2(...)
		return CLIB.btSoftBody_solveClusters2(self.ptr, ...)
	end
	function META:SetVolumeMass(...)
		return CLIB.btSoftBody_setVolumeMass(self.ptr, ...)
	end
	function META:SetWindVelocity(...)
		return CLIB.btSoftBody_setWindVelocity(self.ptr, ...)
	end
	function META:SetVelocity(...)
		return CLIB.btSoftBody_setVelocity(self.ptr, ...)
	end
	function META:SetTotalMass(...)
		return CLIB.btSoftBody_setTotalMass(self.ptr, ...)
	end
	function META:SetTag(...)
		return CLIB.btSoftBody_setTag(self.ptr, ...)
	end
	function META:SetSolver(...)
		return CLIB.btSoftBody_setSolver(self.ptr, ...)
	end
	function META:SetRestLengthScale(...)
		return CLIB.btSoftBody_setRestLengthScale(self.ptr, ...)
	end
	function META:SetPose(...)
		return CLIB.btSoftBody_setPose(self.ptr, ...)
	end
	function META:SetMass(...)
		return CLIB.btSoftBody_setMass(self.ptr, ...)
	end
	function META:SetInitialWorldTransform(...)
		return CLIB.btSoftBody_setInitialWorldTransform(self.ptr, ...)
	end
	function META:Scale(...)
		return CLIB.btSoftBody_scale(self.ptr, ...)
	end
	function META:Rotate(...)
		return CLIB.btSoftBody_rotate(self.ptr, ...)
	end
	function META:ResetLinkRestLengths(...)
		return CLIB.btSoftBody_resetLinkRestLengths(self.ptr, ...)
	end
	function META:ReleaseClusters(...)
		return CLIB.btSoftBody_releaseClusters(self.ptr, ...)
	end
	function META:ReleaseCluster(...)
		return CLIB.btSoftBody_releaseCluster(self.ptr, ...)
	end
	function META:Refine(...)
		return CLIB.btSoftBody_refine(self.ptr, ...)
	end
	function META:RayTest(...)
		return CLIB.btSoftBody_rayTest(self.ptr, ...)
	end
	function META:PrepareClusters(...)
		return CLIB.btSoftBody_prepareClusters(self.ptr, ...)
	end
	function META:PointersToIndices(...)
		return CLIB.btSoftBody_pointersToIndices(self.ptr, ...)
	end
	function META:IntegrateMotion(...)
		return CLIB.btSoftBody_integrateMotion(self.ptr, ...)
	end
	function META:InitDefaults(...)
		return CLIB.btSoftBody_initDefaults(self.ptr, ...)
	end
	function META:IndicesToPointers(...)
		return CLIB.btSoftBody_indicesToPointers(self.ptr, ...)
	end
	function META:GetUserIndexMapping(...)
		return CLIB.btSoftBody_getUserIndexMapping(self.ptr, ...)
	end
	function META:GetTimeacc(...)
		return CLIB.btSoftBody_getTimeacc(self.ptr, ...)
	end
	function META:AppendNote5(...)
		return CLIB.btSoftBody_appendNote5(self.ptr, ...)
	end
	function META:GetSoftBodySolver(...)
		return CLIB.btSoftBody_getSoftBodySolver(self.ptr, ...)
	end
	function META:GetScontacts(...)
		return CLIB.btSoftBody_getScontacts(self.ptr, ...)
	end
	function META:GetRestLengthScale(...)
		return CLIB.btSoftBody_getRestLengthScale(self.ptr, ...)
	end
	function META:GetRcontacts(...)
		return CLIB.btSoftBody_getRcontacts(self.ptr, ...)
	end
	function META:GetPose(...)
		return CLIB.btSoftBody_getPose(self.ptr, ...)
	end
	function META:GetNdbvt(...)
		return CLIB.btSoftBody_getNdbvt(self.ptr, ...)
	end
	function META:GetMaterials(...)
		return CLIB.btSoftBody_getMaterials(self.ptr, ...)
	end
	function META:GetMass(...)
		return CLIB.btSoftBody_getMass(self.ptr, ...)
	end
	function META:GetJoints(...)
		return CLIB.btSoftBody_getJoints(self.ptr, ...)
	end
	function META:GetInitialWorldTransform(...)
		return CLIB.btSoftBody_getInitialWorldTransform(self.ptr, ...)
	end
	function META:GetFdbvt(...)
		return CLIB.btSoftBody_getFdbvt(self.ptr, ...)
	end
	function META:GetCollisionDisabledObjects(...)
		return CLIB.btSoftBody_getCollisionDisabledObjects(self.ptr, ...)
	end
	function META:GetClusterConnectivity(...)
		return CLIB.btSoftBody_getClusterConnectivity(self.ptr, ...)
	end
	function META:GetBounds(...)
		return CLIB.btSoftBody_getBounds(self.ptr, ...)
	end
	function META:GetAnchors(...)
		return CLIB.btSoftBody_getAnchors(self.ptr, ...)
	end
	function META:GenerateClusters2(...)
		return CLIB.btSoftBody_generateClusters2(self.ptr, ...)
	end
	function META:GenerateClusters(...)
		return CLIB.btSoftBody_generateClusters(self.ptr, ...)
	end
	function META:CutLink2(...)
		return CLIB.btSoftBody_cutLink2(self.ptr, ...)
	end
	function META:ClusterVImpulse(...)
		return CLIB.btSoftBody_clusterVImpulse(self.ptr, ...)
	end
	function META:ClusterVAImpulse(...)
		return CLIB.btSoftBody_clusterVAImpulse(self.ptr, ...)
	end
	function META:ClusterDImpulse(...)
		return CLIB.btSoftBody_clusterDImpulse(self.ptr, ...)
	end
	function META:ClusterDCImpulse(...)
		return CLIB.btSoftBody_clusterDCImpulse(self.ptr, ...)
	end
	function META:ClusterDAImpulse(...)
		return CLIB.btSoftBody_clusterDAImpulse(self.ptr, ...)
	end
	function META:ClusterCom2(...)
		return CLIB.btSoftBody_clusterCom2(self.ptr, ...)
	end
	function META:ClusterAImpulse(...)
		return CLIB.btSoftBody_clusterAImpulse(self.ptr, ...)
	end
	function META:CleanupClusters(...)
		return CLIB.btSoftBody_cleanupClusters(self.ptr, ...)
	end
	function META:CheckLink2(...)
		return CLIB.btSoftBody_checkLink2(self.ptr, ...)
	end
	function META:CheckFace(...)
		return CLIB.btSoftBody_checkFace(self.ptr, ...)
	end
	function META:GetSst(...)
		return CLIB.btSoftBody_getSst(self.ptr, ...)
	end
	function META:AppendNote4(...)
		return CLIB.btSoftBody_appendNote4(self.ptr, ...)
	end
	function META:AppendNote3(...)
		return CLIB.btSoftBody_appendNote3(self.ptr, ...)
	end
	function META:AppendLink2(...)
		return CLIB.btSoftBody_appendLink2(self.ptr, ...)
	end
	function META:AppendLinearJoint2(...)
		return CLIB.btSoftBody_appendLinearJoint2(self.ptr, ...)
	end
	function META:AppendFace(...)
		return CLIB.btSoftBody_appendFace(self.ptr, ...)
	end
	function META:AppendAngularJoint(...)
		return CLIB.btSoftBody_appendAngularJoint(self.ptr, ...)
	end
	function META:AppendAnchor2(...)
		return CLIB.btSoftBody_appendAnchor2(self.ptr, ...)
	end
	function META:AppendAnchor(...)
		return CLIB.btSoftBody_appendAnchor(self.ptr, ...)
	end
	function META:DampClusters(...)
		return CLIB.btSoftBody_dampClusters(self.ptr, ...)
	end
	function META:PredictMotion(...)
		return CLIB.btSoftBody_predictMotion(self.ptr, ...)
	end
	function META:AppendMaterial(...)
		return CLIB.btSoftBody_appendMaterial(self.ptr, ...)
	end
	function META:AppendLink3(...)
		return CLIB.btSoftBody_appendLink3(self.ptr, ...)
	end
	function META:StaticSolve(...)
		return CLIB.btSoftBody_staticSolve(self.ptr, ...)
	end
	function META:GetFaceVertexNormalData(...)
		return CLIB.btSoftBody_getFaceVertexNormalData(self.ptr, ...)
	end
	function META:UpdateLinkConstants(...)
		return CLIB.btSoftBody_updateLinkConstants(self.ptr, ...)
	end
	function META:Upcast(...)
		return CLIB.btSoftBody_upcast(self.ptr, ...)
	end
	function META:AppendAngularJoint4(...)
		return CLIB.btSoftBody_appendAngularJoint4(self.ptr, ...)
	end
	function META:AddForce(...)
		return CLIB.btSoftBody_addForce(self.ptr, ...)
	end
	function META:GetWindVelocity(...)
		return CLIB.btSoftBody_getWindVelocity(self.ptr, ...)
	end
	function META:GetBUpdateRtCst(...)
		return CLIB.btSoftBody_getBUpdateRtCst(self.ptr, ...)
	end
	function META:AppendLinearJoint4(...)
		return CLIB.btSoftBody_appendLinearJoint4(self.ptr, ...)
	end
	function META:ApplyForces(...)
		return CLIB.btSoftBody_applyForces(self.ptr, ...)
	end
	function META:ClusterImpulse(...)
		return CLIB.btSoftBody_clusterImpulse(self.ptr, ...)
	end
	function META:UpdateArea(...)
		return CLIB.btSoftBody_updateArea(self.ptr, ...)
	end
	function META:AppendLinearJoint(...)
		return CLIB.btSoftBody_appendLinearJoint(self.ptr, ...)
	end
	function META:GetTetraVertexData(...)
		return CLIB.btSoftBody_getTetraVertexData(self.ptr, ...)
	end
	function META:AppendLink(...)
		return CLIB.btSoftBody_appendLink(self.ptr, ...)
	end
	function META:UpdateConstants(...)
		return CLIB.btSoftBody_updateConstants(self.ptr, ...)
	end
	function META:GetCfg(...)
		return CLIB.btSoftBody_getCfg(self.ptr, ...)
	end
	function META:SetBUpdateRtCst(...)
		return CLIB.btSoftBody_setBUpdateRtCst(self.ptr, ...)
	end
	function META:UpdatePose(...)
		return CLIB.btSoftBody_updatePose(self.ptr, ...)
	end
	function META:SetWorldInfo(...)
		return CLIB.btSoftBody_setWorldInfo(self.ptr, ...)
	end
	function META:InitializeFaceTree(...)
		return CLIB.btSoftBody_initializeFaceTree(self.ptr, ...)
	end
	function META:GetFaces(...)
		return CLIB.btSoftBody_getFaces(self.ptr, ...)
	end
	function META:GetLinkVertexNormalData(...)
		return CLIB.btSoftBody_getLinkVertexNormalData(self.ptr, ...)
	end
	function META:AddAeroForceToNode(...)
		return CLIB.btSoftBody_addAeroForceToNode(self.ptr, ...)
	end
	function META:AddForce2(...)
		return CLIB.btSoftBody_addForce2(self.ptr, ...)
	end
	function META:DefaultCollisionHandler(...)
		return CLIB.btSoftBody_defaultCollisionHandler(self.ptr, ...)
	end
	function META:EvaluateCom(...)
		return CLIB.btSoftBody_evaluateCom(self.ptr, ...)
	end
	function META:ClusterVelocity(...)
		return CLIB.btSoftBody_clusterVelocity(self.ptr, ...)
	end
	function META:AddAeroForceToFace(...)
		return CLIB.btSoftBody_addAeroForceToFace(self.ptr, ...)
	end
	function META:AppendNode(...)
		return CLIB.btSoftBody_appendNode(self.ptr, ...)
	end
	function META:CheckContact(...)
		return CLIB.btSoftBody_checkContact(self.ptr, ...)
	end
	function META:AppendTetra2(...)
		return CLIB.btSoftBody_appendTetra2(self.ptr, ...)
	end
	function META:GetCdbvt(...)
		return CLIB.btSoftBody_getCdbvt(self.ptr, ...)
	end
end
do -- btDbvtNode
	local META = {}
	library.metatables.btDbvtNode = META
	META.__index = function(s, k) return META[k] end
	function library.CreateDbvtNode(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btDbvtNode_new(...)
		return self
	end
	function META:SetParent(...)
		return CLIB.btDbvtNode_setParent(self.ptr, ...)
	end
	function META:Isleaf(...)
		return CLIB.btDbvtNode_isleaf(self.ptr, ...)
	end
	function META:Delete(...)
		return CLIB.btDbvtNode_delete(self.ptr, ...)
	end
	function META:GetParent(...)
		return CLIB.btDbvtNode_getParent(self.ptr, ...)
	end
	function META:Isinternal(...)
		return CLIB.btDbvtNode_isinternal(self.ptr, ...)
	end
	function META:GetVolume(...)
		return CLIB.btDbvtNode_getVolume(self.ptr, ...)
	end
	function META:GetChilds(...)
		return CLIB.btDbvtNode_getChilds(self.ptr, ...)
	end
	function META:GetData(...)
		return CLIB.btDbvtNode_getData(self.ptr, ...)
	end
	function META:SetDataAsInt(...)
		return CLIB.btDbvtNode_setDataAsInt(self.ptr, ...)
	end
	function META:GetDataAsInt(...)
		return CLIB.btDbvtNode_getDataAsInt(self.ptr, ...)
	end
	function META:SetData(...)
		return CLIB.btDbvtNode_setData(self.ptr, ...)
	end
end
do -- btBvhTree
	local META = {}
	library.metatables.btBvhTree = META
	META.__index = function(s, k) return META[k] end
	function library.CreateBvhTree(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btBvhTree_new(...)
		return self
	end
	function META:ClearNodes(...)
		return CLIB.btBvhTree_clearNodes(self.ptr, ...)
	end
	function META:GetNodeBound(...)
		return CLIB.btBvhTree_getNodeBound(self.ptr, ...)
	end
	function META:Delete(...)
		return CLIB.btBvhTree_delete(self.ptr, ...)
	end
	function META:SetNodeBound(...)
		return CLIB.btBvhTree_setNodeBound(self.ptr, ...)
	end
	function META:GetRightNode(...)
		return CLIB.btBvhTree_getRightNode(self.ptr, ...)
	end
	function META:GetEscapeNodeIndex(...)
		return CLIB.btBvhTree_getEscapeNodeIndex(self.ptr, ...)
	end
	function META:GetNodeCount(...)
		return CLIB.btBvhTree_getNodeCount(self.ptr, ...)
	end
	function META:IsLeafNode(...)
		return CLIB.btBvhTree_isLeafNode(self.ptr, ...)
	end
	function META:GetLeftNode(...)
		return CLIB.btBvhTree_getLeftNode(self.ptr, ...)
	end
	function META:GetNodeData(...)
		return CLIB.btBvhTree_getNodeData(self.ptr, ...)
	end
end
do -- btPairSet
	local META = {}
	library.metatables.btPairSet = META
	META.__index = function(s, k) return META[k] end
	function library.CreatePairSet(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btPairSet_new(...)
		return self
	end
	function META:Delete(...)
		return CLIB.btPairSet_delete(self.ptr, ...)
	end
end
do -- btElement
	local META = {}
	library.metatables.btElement = META
	META.__index = function(s, k) return META[k] end
	function library.CreateElement(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btElement_new(...)
		return self
	end
	function META:Delete(...)
		return CLIB.btElement_delete(self.ptr, ...)
	end
	function META:SetSz(...)
		return CLIB.btElement_setSz(self.ptr, ...)
	end
	function META:SetId(...)
		return CLIB.btElement_setId(self.ptr, ...)
	end
	function META:GetSz(...)
		return CLIB.btElement_getSz(self.ptr, ...)
	end
	function META:GetId(...)
		return CLIB.btElement_getId(self.ptr, ...)
	end
end
do -- btChunk
	local META = {}
	library.metatables.btChunk = META
	META.__index = function(s, k) return META[k] end
	function library.CreateChunk(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btChunk_new(...)
		return self
	end
	function META:GetNumber(...)
		return CLIB.btChunk_getNumber(self.ptr, ...)
	end
	function META:Delete(...)
		return CLIB.btChunk_delete(self.ptr, ...)
	end
	function META:SetOldPtr(...)
		return CLIB.btChunk_setOldPtr(self.ptr, ...)
	end
	function META:SetNumber(...)
		return CLIB.btChunk_setNumber(self.ptr, ...)
	end
	function META:GetOldPtr(...)
		return CLIB.btChunk_getOldPtr(self.ptr, ...)
	end
	function META:GetChunkCode(...)
		return CLIB.btChunk_getChunkCode(self.ptr, ...)
	end
	function META:SetChunkCode(...)
		return CLIB.btChunk_setChunkCode(self.ptr, ...)
	end
	function META:GetLength(...)
		return CLIB.btChunk_getLength(self.ptr, ...)
	end
	function META:SetLength(...)
		return CLIB.btChunk_setLength(self.ptr, ...)
	end
end
do -- btAABB
	local META = {}
	library.metatables.btAABB = META
	META.__index = function(s, k) return META[k] end
	function library.CreateAABB(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btAABB_new2(...)
		return self
	end
	function library.CreateAABB2(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btAABB_new3(...)
		return self
	end
	function library.CreateAABB3(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btAABB_new4(...)
		return self
	end
	function library.CreateAABB4(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btAABB_new(...)
		return self
	end
	function library.CreateAABB5(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btAABB_new5(...)
		return self
	end
	function META:Invalidate(...)
		return CLIB.btAABB_invalidate(self.ptr, ...)
	end
	function META:GetMax(...)
		return CLIB.btAABB_getMax(self.ptr, ...)
	end
	function META:Delete(...)
		return CLIB.btAABB_delete(self.ptr, ...)
	end
	function META:SetMax(...)
		return CLIB.btAABB_setMax(self.ptr, ...)
	end
	function META:Merge(...)
		return CLIB.btAABB_merge(self.ptr, ...)
	end
	function META:SetMin(...)
		return CLIB.btAABB_setMin(self.ptr, ...)
	end
	function META:GetMin(...)
		return CLIB.btAABB_getMin(self.ptr, ...)
	end
end
do -- btDbvt
	local META = {}
	library.metatables.btDbvt = META
	META.__index = function(s, k) return META[k] end
	function META:Clone(...)
		return CLIB.btDbvt_clone(self.ptr, ...)
	end
	function META:Update3(...)
		return CLIB.btDbvt_update3(self.ptr, ...)
	end
	function META:GetFree(...)
		return CLIB.btDbvt_getFree(self.ptr, ...)
	end
	function META:SetRoot(...)
		return CLIB.btDbvt_setRoot(self.ptr, ...)
	end
	function META:GetOpath(...)
		return CLIB.btDbvt_getOpath(self.ptr, ...)
	end
	function META:RayTest(...)
		return CLIB.btDbvt_rayTest(self.ptr, ...)
	end
	function META:Maxdepth(...)
		return CLIB.btDbvt_maxdepth(self.ptr, ...)
	end
	function META:Update(...)
		return CLIB.btDbvt_update(self.ptr, ...)
	end
	function META:GetLeaves(...)
		return CLIB.btDbvt_getLeaves(self.ptr, ...)
	end
	function META:Update2(...)
		return CLIB.btDbvt_update2(self.ptr, ...)
	end
	function META:Update5(...)
		return CLIB.btDbvt_update5(self.ptr, ...)
	end
	function META:Insert(...)
		return CLIB.btDbvt_insert(self.ptr, ...)
	end
	function META:ExtractLeaves(...)
		return CLIB.btDbvt_extractLeaves(self.ptr, ...)
	end
	function META:Nearest(...)
		return CLIB.btDbvt_nearest(self.ptr, ...)
	end
	function META:OptimizeBottomUp(...)
		return CLIB.btDbvt_optimizeBottomUp(self.ptr, ...)
	end
	function META:Empty(...)
		return CLIB.btDbvt_empty(self.ptr, ...)
	end
	function META:OptimizeIncremental(...)
		return CLIB.btDbvt_optimizeIncremental(self.ptr, ...)
	end
	function META:SetOpath(...)
		return CLIB.btDbvt_setOpath(self.ptr, ...)
	end
	function META:GetLkhd(...)
		return CLIB.btDbvt_getLkhd(self.ptr, ...)
	end
	function META:Clone2(...)
		return CLIB.btDbvt_clone2(self.ptr, ...)
	end
	function META:SetLkhd(...)
		return CLIB.btDbvt_setLkhd(self.ptr, ...)
	end
	function META:Clear(...)
		return CLIB.btDbvt_clear(self.ptr, ...)
	end
	function META:Update4(...)
		return CLIB.btDbvt_update4(self.ptr, ...)
	end
	function META:Remove(...)
		return CLIB.btDbvt_remove(self.ptr, ...)
	end
	function META:Allocate(...)
		return CLIB.btDbvt_allocate(self.ptr, ...)
	end
	function META:SetLeaves(...)
		return CLIB.btDbvt_setLeaves(self.ptr, ...)
	end
	function META:GetRoot(...)
		return CLIB.btDbvt_getRoot(self.ptr, ...)
	end
	function META:OptimizeTopDown(...)
		return CLIB.btDbvt_optimizeTopDown(self.ptr, ...)
	end
	function META:Benchmark(...)
		return CLIB.btDbvt_benchmark(self.ptr, ...)
	end
	function META:Update6(...)
		return CLIB.btDbvt_update6(self.ptr, ...)
	end
	function META:OptimizeTopDown2(...)
		return CLIB.btDbvt_optimizeTopDown2(self.ptr, ...)
	end
	function META:Write(...)
		return CLIB.btDbvt_write(self.ptr, ...)
	end
	function META:RayTestInternal(...)
		return CLIB.btDbvt_rayTestInternal(self.ptr, ...)
	end
	function META:SetFree(...)
		return CLIB.btDbvt_setFree(self.ptr, ...)
	end
	function META:CountLeaves(...)
		return CLIB.btDbvt_countLeaves(self.ptr, ...)
	end
	function META:GetStkStack(...)
		return CLIB.btDbvt_getStkStack(self.ptr, ...)
	end
end
do -- btFace
	local META = {}
	library.metatables.btFace = META
	META.__index = function(s, k) return META[k] end
	function library.CreateFace(...)
		local self = setmetatable({}, META)
		self.ptr = CLIB.btFace_new(...)
		return self
	end
	function META:GetPlane(...)
		return CLIB.btFace_getPlane(self.ptr, ...)
	end
	function META:GetIndices(...)
		return CLIB.btFace_getIndices(self.ptr, ...)
	end
	function META:Delete(...)
		return CLIB.btFace_delete(self.ptr, ...)
	end
end
library.clib = CLIB
return library
