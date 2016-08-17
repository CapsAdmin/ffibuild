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
float(btRotationalLimitMotor_getLimitSoftness)(void*);
void(btRigidBody_computeGyroscopicImpulseImplicit_World)(void*,float,void*);
void*(btWorldImporter_createGearConstraint)(void*,void*,void*,const void*,const void*,float);
bool(btRotationalLimitMotor2_getSpringStiffnessLimited)(void*);
void(btGImpactCollisionAlgorithm_gimpact_vs_concave)(void*,const void*,const void*,const void*,const void*,bool);
void*(btTriangleIndexVertexArray_getIndexedMeshArray)(void*);
void(btCollisionConfiguration_delete)(void*);
const void*(btCollisionWorld_LocalConvexResult_getHitCollisionObject)(void*);
void(btGeneric6DofSpring2Constraint_setStiffness2)(void*,int,float,bool);
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
void(btCollisionWorld_convexSweepTest)(void*,const void*,const void*,const void*,void*);
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
void(btCollisionShape_setUserPointer)(void*,void*);
void*(btBvhTriangleMeshShape_new2)(void*,bool,bool);
void(btCollisionWorld_RayResultCallback_setCollisionFilterMask)(void*,short);
void*(btSoftBody_Feature_getMaterial)(void*);
void(btGeneric6DofSpring2Constraint_setLimitReversed)(void*,int,float,float);
const void*(btManifoldResult_getBody0Wrap)(void*);
void*(btQuantizedBvh_getLeafNodeArray)(void*);
void*(btCollisionAlgorithmCreateFunc_CreateCollisionAlgorithm)(void*,void*,const void*,const void*);
float(btSliderConstraint_getSoftnessDirLin)(void*);
const void*(btConvexConvexAlgorithm_getManifold)(void*);
float(btRotationalLimitMotor2_getTargetVelocity)(void*);
const void*(btGImpactBvh_get_node_pointer)(void*);
void(btRotationalLimitMotor2_setSpringStiffnessLimited)(void*,bool);
void(btSoftBody_Body_applyAImpulse)(void*,const void*);
void*(btBroadphaseProxy_getClientObject)(void*);
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
void(btAngularLimit_set3)(void*,float,float,float,float);
float(btSoftBody_AJoint_IControl_Speed)(void*,void*,float);
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
btAlignedObjectArray_const_btCollisionObjectPtr*(btCollisionWorld_AllHitsRayResultCallback_getCollisionObjects)(void*);
int(btConvexShape_getNumPreferredPenetrationDirections)(void*);
void(btDefaultCollisionConfiguration_setPlaneConvexMultipointIterations2)(void*,int);
void(btSoftBody_appendFace4)(void*,int,int,int);
void*(btSoftBody_Impulse_operator_m)(void*,float);
void(btConvexPolyhedron_initialize)(void*);
void(btDbvtAabbMm_Maxs)(void*,void*);
int(btSliderConstraint_getFlags)(void*);
void*(btMultiBodyJointMotor_new)(void*,int,float,float);
void*(btBroadphaseProxy_getMultiSapParentProxy)(void*);
void(btSimulationIslandManager_IslandCallback_delete)(void*);
void(btCapsuleShape_deSerializeFloat)(void*,void*);
void(btSoftBody_Node_setN)(void*,const void*);
void(btCollisionWorld_delete)(void*);
void*(btHingeAccumulatedAngleConstraint_new6)(void*,void*,const void*,const void*,bool);
void(btDbvt_update3)(void*,void*,int);
void*(btConvexPolyhedron_new)();
void(btBvhTriangleMeshShape_serializeSingleTriangleInfoMap)(void*,void*);
const void*(btGImpactQuantizedBvh_get_node_pointer2)(void*,int);
void(btTriangleShapeEx_buildTriPlane)(void*,void*);
int(btHingeConstraint_getSolveLimit)(void*);
void*(btDbvt_getFree)(void*);
void(btCollisionObject_setAnisotropicFriction)(void*,const void*);
void*(btCompoundShape_new2)(bool);
void(btPoint2PointConstraint_setPivotA)(void*,const void*);
void*(btDbvtAabbMm_new)();
void*(btGeneric6DofSpring2Constraint_new4)(void*,const void*,int);
void*(btConvexPointCloudShape_new)();
void*(btBroadphaseInterface_createProxy)(void*,const void*,const void*,int,void*,short,short,void*,void*);
float(btContactSolverInfoData_getFriction)(void*);
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
void(btDispatcherInfo_setTimeStep)(void*,float);
void(btBvhTree_getNodeBound)(void*,int,void*);
int(btPersistentManifold_addManifoldPoint2)(void*,const void*,bool);
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
void(btDynamicsWorld_addConstraint)(void*,void*);
int(btGImpactCollisionAlgorithm_getFace1)(void*);
void(btCollisionWorld_ConvexResultCallback_setCollisionFilterMask)(void*,short);
bool(btMultiBody_getCanSleep)(void*);
float(btSliderConstraint_getSoftnessOrthoAng)(void*);
int(btTypedConstraint_getUid)(void*);
void(btMultiBodyConstraintSolver_solveMultiBodyGroup)(void*,void**,int,void**,int,void**,int,void**,int,const void*,void*,void*);
void(btGeneric6DofConstraint_getInfo1NonVirtual)(void*,void*);
void*(btChunk_new)();
void(btWheelInfoConstructionInfo_setWheelsDampingRelaxation)(void*,float);
unsigned int(btDbvt_getOpath)(void*);
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
void(btGeneric6DofSpring2Constraint_setLinearUpperLimit)(void*,const void*);
void*(btSoftBody_SContact_getFace)(void*);
void(btSoftBody_Cluster_setContainsAnchor)(void*,bool);
bool(btUsageBitfield_getUnused3)(void*);
void(btDbvt_rayTest)(const void*,const void*,const void*,void*);
void(btContactSolverInfoData_setTimeStep)(void*,float);
void(btQuantizedBvh_reportRayOverlappingNodex)(void*,void*,const void*,const void*);
int(btCollisionObject_getCollisionFlags)(void*);
void(btDiscreteDynamicsWorld_synchronizeSingleMotionState)(void*,void*);
bool(btOverlappingPairCache_hasDeferredRemoval)(void*);
short(btCollisionWorld_RayResultCallback_getCollisionFilterGroup)(void*);
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
void(btSoftBody_appendAnchor4)(void*,int,void*);
int(btAlignedObjectArray_btSoftBody_Tetra_size)(void*);
void(btSoftBody_Impulse_delete)(void*);
int(btGImpactBvh_getEscapeNodeIndex)(void*,int);
float(btSliderConstraint_getDampingLimLin)(void*);
void*(btDefaultCollisionConfiguration_new2)(const void*);
void*(btSoftRigidDynamicsWorld_new)(void*,void*,void*,void*);
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
void*(btDbvt_insert)(void*,const void*,void*);
void(btNNCGConstraintSolver_setOnlyForNoneContact)(void*,bool);
void*(btDbvt_sStkNP_new)(const void*,unsigned int);
void(btAlignedObjectArray_btSoftBody_Tetra_resizeNoInitialize)(void*,int);
void(btSubSimplexClosestResult_setBarycentricCoordinates3)(void*,float,float);
void(btQuantizedBvh_delete)(void*);
float(btRaycastVehicle_getCurrentSpeedKmHour)(void*);
void(btGeneric6DofConstraint_setAxis)(void*,const void*,const void*);
void(btDiscreteCollisionDetectorInterface_ClosestPointInput_setTransformB)(void*,const void*);
const char*(btCollisionShape_serialize)(void*,void*,void*);
void*(btCompoundCollisionAlgorithm_SwappedCreateFunc_new)();
void(btCollisionWorld_ClosestConvexResultCallback_setHitCollisionObject)(void*,const void*);
void(btCollisionWorld_rayTest)(void*,const void*,const void*,void*);
void*(btConvexPolyhedron_getVertices)(void*);
int(btGImpactQuantizedBvh_getEscapeNodeIndex)(void*,int);
void*(btConvexConvexAlgorithm_CreateFunc_getSimplexSolver)(void*);
void*(btCylinderShape_new)(const void*);
int(btSoftBody_Impulse_getAsDrift)(void*);
void(btStridingMeshInterface_calculateAabbBruteForce)(void*,void*,void*);
void(btMultiBodySolverConstraint_setAngularComponentB)(void*,const void*);
void(btContactSolverInfoData_setTau)(void*,float);
void*(btWorldImporter_createConeShapeY)(void*,float,float);
float(btTriangleInfoMap_getMaxEdgeAngleThreshold)(void*);
void(btRotationalLimitMotor_setAccumulatedImpulse)(void*,float);
void*(btConvexHullShape_new2)(const float*);
bool(btSubSimplexClosestResult_isValid)(void*);
void*(btRigidBody_new)(const void*);
int(btRaycastVehicle_getUpAxis)(void*);
float(btAngularLimit_getBiasFactor)(void*);
float(btRigidBody_btRigidBodyConstructionInfo_getAdditionalAngularDampingFactor)(void*);
void(btSliderConstraint_getInfo2NonVirtual)(void*,void*,const void*,const void*,const void*,const void*,float,float);
void(btBroadphaseProxy_getAabbMin)(void*,void*);
unsigned short(btAxisSweep3_addHandle)(void*,const void*,const void*,void*,short,short,void*,void*);
void*(btOverlappingPairCache_getOverlappingPairArray)(void*);
void(btOverlappingPairCallback_delete)(void*);
void*(btMultiBody_getBaseCollider)(void*);
int(btSoftBody_CJoint_getMaxlife)(void*);
void*(btMultiBodyJointMotor_new2)(void*,int,int,float,float);
void(btMultibodyLink_setCfgOffset)(void*,int);
float(btSoftBody_Cluster_getIdmass)(void*);
void*(btBvhTriangleMeshShape_getOptimizedBvh)(void*);
void*(btIDebugDrawWrapper_getGCHandle)(void*);
float(btSoftBody_Config_getKDG)(void*);
void*(btGImpactBvh_getPrimitiveManager)(void*);
const unsigned char*(btGImpactMeshShapePart_TrimeshPrimitiveManager_getIndexbase)(void*);
void(btGImpactCollisionAlgorithm_setFace1)(void*,int);
void(btTypedConstraint_btConstraintInfo1_setNumConstraintRows)(void*,int);
void(btBox2dShape_getPlaneEquation)(void*,void*,int);
void(btPoint2PointConstraint_setUseSolveConstraintObsolete)(void*,bool);
void(btDynamicsWorld_setInternalTickCallback3)(void*,void*,void*,bool);
int(btGImpactMeshShapePart_TrimeshPrimitiveManager_getIndicestype)(void*);
float(btUniversalConstraint_getAngle1)(void*);
void(btBroadphaseProxy_setClientObject)(void*,void*);
void(btDiscreteCollisionDetectorInterface_getClosestPoints2)(void*,const void*,void*,void*,bool);
void(btSoftRigidDynamicsWorld_removeSoftBody)(void*,void*);
int(btGjkPairDetector_getCurIter)(void*);
void(btSoftBody_appendLink7)(void*,void*,void*);
void(btManifoldPoint_delete)(void*);
bool(btAABB_collide_ray)(void*,const void*,const void*);
void(btSoftBody_appendNote)(void*,const char*,const void*,void*);
void*(btGjkConvexCast_new)(const void*,const void*,void*);
void(btKinematicCharacterController_setMaxJumpHeight)(void*,float);
void*(btHingeConstraint_new8)(void*,const void*,bool);
void(btTypedConstraint_delete)(void*);
const void*(btTriangleBuffer_getTriangle)(void*,int);
void(btManifoldPoint_setFrictionCFM)(void*,float);
unsigned int(btDbvtBroadphase_getUpdates_done)(void*);
float(btManifoldPoint_getCombinedFriction)(void*);
void(btConstraintSolver_reset)(void*);
float(btConeTwistConstraint_getSwingSpan1)(void*);
void(btAABB_copy_with_margin)(void*,const void*,float);
void(btDiscreteCollisionDetectorInterface_ClosestPointInput_getTransformA)(void*,void*);
void(btMultiBodySolverConstraint_setLowerLimit)(void*,float);
bool(btCollisionWorld_ConvexResultCallback_needsCollision)(void*,void*);
void(btMultiBodySolverConstraint_setFriction)(void*,float);
void(btRigidBody_btRigidBodyConstructionInfo_delete)(void*);
void(btAlignedObjectArray_btVector3_delete)(void*);
int(btAlignedObjectArray_btSoftBody_MaterialPtr_size)(void*);
void(btUnionFind_Free)(void*);
int(btAlignedObjectArray_btVector3_size)(void*);
int(btSoftBody_sRayCast_getFeature)(void*);
void*(btWorldImporter_createSliderConstraint2)(void*,void*,const void*,bool);
void*(btWorldImporter_createMultiSphereShape)(void*,const void*,const float*,int);
void(btRotationalLimitMotor2_setHiLimit)(void*,float);
void(btMultibodyLink_setCachedWorldTransform)(void*,const void*);
void(btAlignedObjectArray_btVector3_set)(void*,int,const void*);
void(btMultiBody_setupSpherical)(void*,int,float,const void*,int,const void*,const void*,const void*);
void(btSoftBody_Link_setC1)(void*,float);
void(btAlignedObjectArray_btVector3_push_back)(void*,const void*);
void(btDbvt_update)(void*,void*,void*);
void(btSliderConstraint_setRestitutionDirLin)(void*,float);
bool(btMultiBody_getUseGyroTerm)(void*);
void*(btGeneric6DofSpring2Constraint_getRotationalLimitMotor)(void*,int);
void*(btConvexCast_CastResult_new)();
void(btDynamicsWorld_addAction)(void*,void*);
void*(btAlignedObjectArray_btVector3_new)();
void(btConvexInternalShape_setImplicitShapeDimensions)(void*,const void*);
void*(btSphereSphereCollisionAlgorithm_new2)(const void*);
float(btSoftBody_Config_getKVCF)(void*);
void(btVector3_array_at)(const void*,int,void*);
void(btSoftBodyNodePtrArray_set)(void*,void*,int);
bool(btGImpactQuantizedBvh_isLeafNode)(void*,int);
void*(btSoftBodyNodePtrArray_at)(void*,int);
void*(btWheelInfo_RaycastInfo_getGroundObject)(void*);
void*(btDbvtBroadphase_getPaircache)(void*);
void(btRigidBody_getOrientation)(void*,void*);
void(btWorldImporter_delete)(void*);
void(btCollisionObject_activate)(void*);
bool(btDbvtNode_isleaf)(void*);
void(btPersistentManifold_setIndex1a)(void*,int);
void(btRotationalLimitMotor2_delete)(void*);
int(btWorldImporter_getVerboseMode)(void*);
void(btSoftBody_appendAngularJoint3)(void*,const void*,void*);
void(btMultiBodySolverConstraint_delete)(void*);
int(btDefaultCollisionConstructionInfo_getDefaultMaxPersistentManifoldPoolSize)(void*);
void(btTriangleIndexVertexMaterialArray_addMaterialProperties2)(void*,const void*,int);
void(btSoftBodySolver_delete)(void*);
float*(btMultiBody_getJointTorqueMultiDof)(void*,int);
float(btSoftBody_Config_getKAHR)(void*);
void*(btWorldImporter_getRigidBodyByIndex)(void*,int);
void(btCollisionWorld_convexSweepTest2)(void*,const void*,const void*,const void*,void*,float);
int(btWorldImporter_getNumTriangleInfoMaps)(void*);
void*(btWorldImporter_new)(void*);
float(btTranslationalLimitMotor_getDamping)(void*);
void(btSoftBody_Cluster_setLocii)(void*,const void*);
void(btTriangleInfo_setFlags)(void*,int);
int(btWorldImporter_getNumBvhs)(void*);
void*(btSoftBody_getLinks)(void*);
void*(btAxisSweep3_new4)(const void*,const void*,unsigned short,void*,bool);
void(btSoftBody_Body_angularVelocity2)(void*,void*);
void*(btWorldImporter_getConstraintByName)(void*,const char*);
void*(btBroadphasePair_new)();
void(btTranslationalLimitMotor_getNormalCFM)(void*,void*);
void(btSoftBody_Cluster_setImass)(void*,float);
void*(btWorldImporter_getCollisionShapeByName)(void*,const char*);
void*(btMinkowskiPenetrationDepthSolver_new)();
void*(btBU_Simplex1to4_new4)(const void*,const void*,const void*);
bool(btCollisionObject_isKinematicObject)(void*);
void(btGImpactShapeInterface_lockChildShapes)(void*);
void(btBroadphaseProxy_setUniqueId)(void*,int);
void(btSliderConstraint_setRestitutionOrthoLin)(void*,float);
void(btStridingMeshInterface_getPremadeAabb)(void*,void*,void*);
void(btSparseSdf3_GarbageCollect)(void*,int);
float(btSliderConstraint_getAngularPos)(void*);
float(btManifoldPoint_getAppliedImpulse)(void*);
void*(btWorldImporter_createCapsuleShapeX)(void*,float,float);
void*(btContactSolverInfo_new)();
int(btOptimizedBvhNode_getTriangleIndex)(void*);
void(btConeTwistConstraint_setFixThresh)(void*,float);
void(btSoftBody_Body_applyVAImpulse)(void*,const void*);
void*(btWorldImporter_createTriangleMeshContainer)(void*);
void(btCollisionObject_serializeSingleObject)(void*,void*);
const void*(btCollisionWorld_ClosestConvexResultCallback_getHitCollisionObject)(void*);
void(btDbvtBroadphase_setGid)(void*,int);
void*(btWorldImporter_createStridingMeshInterfaceData)(void*,void*);
void*(btWorldImporter_createSphereShape)(void*,float);
void(btSoftBody_appendNote8)(void*,const char*,const void*,const void*,void*,void*,void*);
void*(btWorldImporter_createSliderConstraint)(void*,void*,void*,const void*,const void*,bool);
void(btTypedConstraint_btConstraintInfo2_setConstraintError)(void*,float*);
void(btCollisionWorld_ConvexResultCallback_setClosestHitFraction)(void*,float);
void(btHeightfieldTerrainShape_setUseZigzagSubdivision2)(void*,bool);
void(btMultiBody_setHasSelfCollision)(void*,bool);
float(btSoftBody_Link_getC0)(void*);
void(btSoftBody_Link_setC0)(void*,float);
void(btHinge2Constraint_getAxis2)(void*,void*);
void(btSliderConstraint_setUseFrameOffset)(void*,bool);
float(btCollisionWorld_RayResultCallback_getClosestHitFraction)(void*);
void*(btWorldImporter_createPlaneShape)(void*,const void*,float);
int(btSoftBody_getFaceVertexData)(void*,float*);
void*(btWorldImporter_createOptimizedBvh)(void*);
void*(btWorldImporter_createMeshInterface)(void*,void*);
void*(btWorldImporter_createHingeConstraint4)(void*,void*,const void*,bool);
int(btMaterialProperties_getMaterialType)(void*);
void*(btWorldImporter_createHingeConstraint3)(void*,void*,const void*);
void*(btWorldImporter_createHingeConstraint2)(void*,void*,void*,const void*,const void*,bool);
void*(btWorldImporter_createHingeConstraint)(void*,void*,void*,const void*,const void*);
void(btQuantizedBvh_buildInternal)(void*);
void(btUsageBitfield_setUnused1)(void*,bool);
void(btAlignedObjectArray_btSoftBody_ClusterPtr_push_back)(void*,void*);
void*(btWorldImporter_createGeneric6DofSpring2Constraint)(void*,void*,void*,const void*,const void*,int);
void(btSoftBodyHelpers_Draw)(void*,void*);
float(btHingeConstraint_getLimitBiasFactor)(void*);
float(btSoftBody_Anchor_getInfluence)(void*);
void*(btWorldImporter_createGeneric6DofConstraint)(void*,void*,void*,const void*,const void*,bool);
void*(btWorldImporter_createCylinderShapeY)(void*,float,float);
void*(btWorldImporter_createCylinderShapeX)(void*,float,float);
void*(btDbvtAabbMm_FromCE)(const void*,const void*);
bool(btTypedConstraint_isEnabled)(void*);
float(btConvexSeparatingDistanceUtil_getConservativeSeparatingDistance)(void*);
void(btDbvtBroadphase_setUpdates_done)(void*,unsigned int);
bool(btCollisionShape_isConvex)(void*);
void(btGhostObject_convexSweepTest)(void*,const void*,const void*,const void*,void*);
void(btSoftBody_appendAnchor5)(void*,int,void*,bool);
void*(btWorldImporter_createConvexTriangleMeshShape)(void*,void*);
int(btDbvt_getLeaves)(void*);
int(btMultibodyLink_getJointType)(void*);
void(btTriangleShapeEx_applyTransform)(void*,const void*);
void*(btWorldImporter_createConeTwistConstraint2)(void*,void*,const void*);
void(btGeneric6DofConstraint_setAngularLowerLimit)(void*,const void*);
int(btDbvtBroadphase_getStageCurrent)(void*);
bool*(btTranslationalLimitMotor2_getServoMotor)(void*);
void(btManifoldResult_setPersistentManifold)(void*,void*);
int(btDynamicsWorld_getNumConstraints)(void*);
void*(btCollisionWorld_LocalShapeInfo_new)();
void(btGeneric6DofSpring2Constraint_setAngularLowerLimitReversed)(void*,const void*);
void(btConvexPlaneCollisionAlgorithm_CreateFunc_setNumPerturbationIterations)(void*,int);
bool(btBulletWorldImporter_loadFile)(void*,const char*);
void*(btWorldImporter_createCompoundShape)(void*);
bool(btCollisionWorld_RayResultCallback_hasHit)(void*);
void(btManifoldPoint_setUserPersistentData)(void*,void*);
int(btConvexPointCloudShape_getNumPoints)(void*);
void*(btWorldImporter_createBvhTriangleMeshShape)(void*,void*,void*);
void(btMultibodyLink_setAppliedConstraintTorque)(void*,const void*);
void*(btWorldImporter_createBoxShape)(void*,const void*);
void(btWheelInfo_delete)(void*);
void(btWheelInfo_updateWheel)(void*,const void*,void*);
void(btSoftBody_solveClusters)(const void*);
void(btMultiBodySolverConstraint_setLinkA)(void*,int);
void(btWheelInfo_setWheelsSuspensionForce)(void*,float);
void(btWheelInfo_setWheelsRadius)(void*,float);
void(btWheelInfo_setWheelsDampingRelaxation)(void*,float);
void(btWheelInfo_setWheelsDampingCompression)(void*,float);
void(btWheelInfo_setWheelDirectionCS)(void*,const void*);
void(btMultiBodySolverConstraint_setRelpos1CrossNormal)(void*,const void*);
void(btDbvtBroadphase_setUpdates_call)(void*,unsigned int);
void(btWheelInfo_setSuspensionStiffness)(void*,float);
void(btWheelInfo_setSuspensionRestLength1)(void*,float);
void*(btSoftBody_getClusters)(void*);
void(btUnionFind_unite)(void*,int,int);
void(btWheelInfo_setSuspensionRelativeVelocity)(void*,float);
void(btConeTwistConstraint_setMotorTargetInConstraintSpace)(void*,const void*);
void(btWheelInfo_setSteering)(void*,float);
void(btRigidBody_getVelocityInLocalPoint)(void*,const void*,void*);
void*(btTypedConstraint_getUserConstraintPtr)(void*);
void*(btActionInterfaceWrapper_new)(void*,void*);
void(btTypedConstraint_setEnabled)(void*,bool);
void(btAlignedObjectArray_btSoftBody_Tetra_push_back)(void*,void*);
void(btRigidBody_btRigidBodyConstructionInfo_setLocalInertia)(void*,const void*);
int(btAlignedObjectArray_btSoftBody_ClusterPtr_size)(void*);
void(btWheelInfo_setMaxSuspensionTravelCm)(void*,float);
int(btConeTwistConstraint_getSolveTwistLimit)(void*);
int(btQuantizedBvh_calculateSerializeBufferSizeNew)(void*);
void(btGImpactCompoundShape_addChildShape2)(void*,void*);
void(btWheelInfo_setMaxSuspensionForce)(void*,float);
void(btWheelInfo_setFrictionSlip)(void*,float);
void(btRigidBody_integrateVelocities)(void*,float);
void(btWheelInfo_setDeltaRotation)(void*,float);
void*(btSoftBody_Node_getLeaf)(void*);
void(btDbvt_sStkNPS_setMask)(void*,int);
void(btWheelInfo_setClippedInvContactDotSuspension)(void*,float);
void(btConvexCast_CastResult_drawCoordSystem)(void*,const void*);
void(btWheelInfo_setClientInfo)(void*,void*);
void(btWheelInfo_setChassisConnectionPointCS)(void*,const void*);
void(btWheelInfo_setBrake)(void*,float);
void*(btConvexTriangleMeshShape_new)(void*);
void(btTranslationalLimitMotor_getLowerLimit)(void*,void*);
void(btWheelInfo_getWorldTransform)(void*,void*);
int(btAlignedObjectArray_btCollisionObjectPtr_size)(void*);
float(btWheelInfo_getWheelsRadius)(void*);
float(btAngularLimit_getError)(void*);
void(btDiscreteCollisionDetectorInterface_Result_setShapeIdentifiersB)(void*,int,int);
int(btGImpactMeshShapePart_getVertexCount)(void*);
void*(btConvexHullShape_new3)(const float*,int);
void(btSoftBodyConcaveCollisionAlgorithm_clearCache)(void*);
void(btSoftBody_Cluster_setIdmass)(void*,float);
void(btMultiBody_setupPrismatic)(void*,int,float,const void*,int,const void*,const void*,const void*,const void*,bool);
float(btSoftBody_getMass)(void*,int);
void(btTriangleMesh_addIndex)(void*,int);
void(btSoftBody_solveConstraints)(void*);
void*(btHingeAccumulatedAngleConstraint_new)(void*,void*,const void*,const void*,const void*,const void*);
void(btActionInterface_delete)(void*);
void*(btGImpactMeshShapePart_TrimeshPrimitiveManager_new3)();
void(btBvhTriangleMeshShape_performRaycast)(void*,void*,const void*,const void*);
void(btSoftBody_clusterAImpulse)(void*,const void*);
void(btCollisionWorld_ContactResultCallback_setCollisionFilterGroup)(void*,short);
void(btCollisionShape_getAabb)(void*,const void*,void*,void*);
void(btSoftBody_appendFace2)(void*,int);
void(btBroadphaseRayCallback_setLambda_max)(void*,float);
float(btWheelInfo_getSuspensionRelativeVelocity)(void*);
float(btWheelInfo_getRollInfluence)(void*);
float(btWheelInfo_getSkidInfo)(void*);
float(btWheelInfo_getRotation)(void*);
void*(btTriangleBuffer_new)();
float(btWheelInfo_getSteering)(void*);
void(btSliderConstraint_getCalculatedTransformA)(void*,void*);
int(btCollisionObject_getCompanionId)(void*);
void*(btWheelInfo_getRaycastInfo)(void*);
float(btWheelInfo_getMaxSuspensionTravelCm)(void*);
void*(btSimulationIslandManager_getUnionFind)(void*);
void(btWheelInfo_setWorldTransform)(void*,const void*);
float(btWheelInfo_getFrictionSlip)(void*);
void(btSliderConstraint_setDampingDirLin)(void*,float);
void(btGeometryUtil_getPlaneEquationsFromVertices)(void*,void*);
const char*(btCollisionObject_serialize)(void*,void*,void*);
void*(btAngularLimit_new)();
void(btConeShape_setHeight)(void*,float);
const void*(btBoxBoxDetector_getBox2)(void*);
float(btTriangleInfoMap_getEdgeDistanceThreshold)(void*);
void(btWheelInfo_getChassisConnectionPointCS)(void*,void*);
void*(btDefaultMotionState_new2)(const void*);
void(btDbvtNode_delete)(void*);
void(btManifoldPoint_getNormalWorldOnB)(void*,void*);
void(btMultiBody_setCompanionId)(void*,int);
void(btContactSolverInfoData_setLinearSlop)(void*,float);
bool(btWheelInfo_getBIsFrontWheel)(void*);
void(btSoftBody_Anchor_setC1)(void*,const void*);
float(btMultiBody_getLinearDamping)(void*);
float(btPersistentManifold_getContactBreakingThreshold)(void*);
void(btWheelInfo_RaycastInfo_delete)(void*);
void(btCollisionObject_setDeactivationTime)(void*,float);
void(btWheelInfo_RaycastInfo_setWheelDirectionWS)(void*,const void*);
void(btCollisionObject_getInterpolationWorldTransform)(void*,void*);
void(btWheelInfo_RaycastInfo_setWheelAxleWS)(void*,const void*);
bool(btBulletWorldImporter_loadFile2)(void*,const char*,const char*);
void(btCollisionShape_calculateTemporalAabb)(void*,const void*,const void*,const void*,float,void*,void*);
void(btRigidBody_setAngularFactor2)(void*,float);
void(btWheelInfo_RaycastInfo_setSuspensionLength)(void*,float);
void(btWheelInfo_RaycastInfo_setIsInContact)(void*,bool);
const void*(btMinkowskiSumShape_getShapeA)(void*);
void(btWheelInfo_RaycastInfo_setHardPointWS)(void*,const void*);
void*(btTriangleIndexVertexMaterialArray_new2)(int,int*,int,int,float*,int,int,unsigned char*,int,int*,int);
void(btSoftBody_appendNote7)(void*,const char*,const void*,const void*,void*,void*);
void(btWheelInfo_RaycastInfo_setGroundObject)(void*,void*);
void(btWheelInfo_RaycastInfo_setContactPointWS)(void*,const void*);
void(btRotationalLimitMotor2_testLimitValue)(void*,float);
void(btWheelInfo_RaycastInfo_setContactNormalWS)(void*,const void*);
void(btSliderConstraint_getFrameOffsetA)(void*,void*);
void(btWheelInfo_RaycastInfo_getWheelDirectionWS)(void*,void*);
void(btDbvtBroadphase_setAabbForceUpdate)(void*,void*,const void*,const void*,void*);
int(btConvexPlaneCollisionAlgorithm_CreateFunc_getMinimumPointsPerturbationThreshold)(void*);
void(btMultiBodySolverConstraint_setSolverBodyIdA)(void*,int);
float(btWheelInfo_RaycastInfo_getSuspensionLength)(void*);
bool(btWheelInfo_RaycastInfo_getIsInContact)(void*);
void(btWheelInfo_RaycastInfo_getHardPointWS)(void*,void*);
bool(btDbvt_update5)(void*,void*,void*,const void*);
void*(btCompoundShapeChild_array_at)(void*,int);
void(btSoftBody_appendLinearJoint2)(void*,const void*);
void(btWheelInfo_RaycastInfo_getContactNormalWS)(void*,void*);
void(btChunk_setDna_nr)(void*,int);
int(btTranslationalLimitMotor_testLimitValue)(void*,int,float);
void*(btBoxShape_new)(const void*);
void*(btGhostObject_getOverlappingObject)(void*,int);
float(btMultiBody_getMaxAppliedImpulse)(void*);
void(btWheelInfoConstructionInfo_setWheelRadius)(void*,float);
void*(btContactSolverInfoData_new)();
void(btWheelInfoConstructionInfo_setWheelDirectionCS)(void*,const void*);
void(btWheelInfoConstructionInfo_setWheelAxleCS)(void*,const void*);
void(btMultiBodySolverConstraint_getAngularComponentA)(void*,void*);
void*(btHeightfieldTerrainShape_new2)(int,int,const void*,float,int,bool,bool);
void(btAlignedObjectArray_btIndexedMesh_resizeNoInitialize)(void*,int);
void(btWheelInfoConstructionInfo_setSuspensionStiffness)(void*,float);
void(btMultiBody_applyDeltaVeeMultiDof)(void*,const float*,float);
void(btWheelInfoConstructionInfo_setSuspensionRestLength)(void*,float);
void*(btCompoundCollisionAlgorithm_new)(const void*,const void*,const void*,bool);
const void*(btCollisionObjectWrapper_getParent)(void*);
void*(btGImpactBvh_getGlobalBox)(void*);
void(btWheelInfoConstructionInfo_setMaxSuspensionTravelCm)(void*,float);
void*(btDbvtBroadphase_getSets)(void*);
void(btWheelInfoConstructionInfo_setMaxSuspensionForce)(void*,float);
void(btWheelInfoConstructionInfo_setFrictionSlip)(void*,float);
void(btRigidBody_setMassProps)(void*,float,const void*);
void(btWheelInfoConstructionInfo_setChassisConnectionCS)(void*,const void*);
void(btWheelInfoConstructionInfo_setBIsFrontWheel)(void*,bool);
void(btManifoldPoint_setCombinedFriction)(void*,float);
float(btWheelInfoConstructionInfo_getWheelsDampingRelaxation)(void*);
void(btMultiBody_stepPositionsMultiDof2)(void*,float,float*);
void(btConvexPolyhedron_setMC)(void*,const void*);
float(btWheelInfoConstructionInfo_getWheelRadius)(void*);
const void*(btPolyhedralConvexShape_getConvexPolyhedron)(void*);
void(btSoftBody_appendAngularJoint)(void*,const void*);
int(btTypedConstraint_getUserConstraintId)(void*);
void(btMotionState_setWorldTransform)(void*,const void*);
void(btSoftBody_appendLink8)(void*,void*,void*,void*);
void(btWheelInfoConstructionInfo_getWheelAxleCS)(void*,void*);
float(btWheelInfoConstructionInfo_getSuspensionStiffness)(void*);
float(btWheelInfoConstructionInfo_getSuspensionRestLength)(void*);
void(btContactSolverInfoData_setDamping)(void*,float);
void(btHingeConstraint_setLimit3)(void*,float,float,float,float);
unsigned int(btQuantizedBvh_getAlignmentSerializationPadding)();
float(btWheelInfoConstructionInfo_getMaxSuspensionForce)(void*);
float(btWheelInfoConstructionInfo_getFrictionSlip)(void*);
float(btSoftBody_Material_getKAST)(void*);
void(btWheelInfoConstructionInfo_getChassisConnectionCS)(void*,void*);
void(btTriangle_getVertex2)(void*,void*);
bool(btWheelInfoConstructionInfo_getBIsFrontWheel)(void*);
void*(btGImpactCompoundShape_getCompoundPrimitiveManager)(void*);
void*(btGImpactCompoundShape_CompoundPrimitiveManager_new3)();
void(btPrimitiveManagerBase_get_primitive_box)(void*,int,void*);
int(btTypedConstraint_btConstraintInfo1_getNumConstraintRows)(void*);
void(btGImpactBvh_getNodeTriangle)(void*,int,void*);
void(btRigidBody_applyTorqueImpulse)(void*,const void*);
bool(btVoronoiSimplexSolver_updateClosestVectorAndPoints)(void*);
int(btPolyhedralConvexShape_getNumPlanes)(void*);
void(btVoronoiSimplexSolver_setNumVertices)(void*,int);
void(btConvexHullShape_addPoint2)(void*,const void*,bool);
void(btCollisionObject_getWorldTransform)(void*,void*);
void(btVoronoiSimplexSolver_setNeedsUpdate)(void*,bool);
void*(btCylinderShapeX_new)(const void*);
void(btDbvtAabbMm_Lengths)(void*,void*);
void(btHingeConstraint_setMotorTarget2)(void*,const void*,float);
void(btMultiBodyDynamicsWorld_clearMultiBodyForces)(void*);
void(btCollisionObject_setInterpolationLinearVelocity)(void*,const void*);
void(btAABB_getMax)(void*,void*);
int(btManifoldPoint_getIndex0)(void*);
void(btMLCPSolverInterface_delete)(void*);
void(btVoronoiSimplexSolver_setLastW)(void*,const void*);
void(btMultibodyLink_setEVector)(void*,const void*);
void(btRigidBody_setAngularFactor)(void*,const void*);
void(btSoftBody_Config_setKVCF)(void*,float);
void(btDbvt_IWriter_WriteLeaf)(void*,const void*,int,int);
void(btVoronoiSimplexSolver_setCachedV)(void*,const void*);
void(btTranslationalLimitMotor2_delete)(void*);
void(btVoronoiSimplexSolver_setCachedP2)(void*,const void*);
void(btMultibodyLink_setDofCount)(void*,int);
void(btAABB_delete)(void*);
void(btVoronoiSimplexSolver_setCachedP1)(void*,const void*);
void(btVoronoiSimplexSolver_setCachedBC)(void*,const void*);
void(btMultiBody_stepPositionsMultiDof)(void*,float);
void*(btSoftBody_AJoint_IControl_new)();
void(btVoronoiSimplexSolver_reset)(void*);
void(btVoronoiSimplexSolver_removeVertex)(void*,int);
void(btVoronoiSimplexSolver_reduceVertices)(void*,const void*);
void*(btSoftBody_ImplicitFnWrapper_new)(void*);
void*(btScaledBvhTriangleMeshShape_new)(void*,const void*);
void(btBvhTree_delete)(void*);
float(btRotationalLimitMotor2_getCurrentLimitErrorHi)(void*);
int(btVoronoiSimplexSolver_pointOutsideOfPlane)(void*,const void*,const void*,const void*,const void*,const void*);
int(btVoronoiSimplexSolver_numVertices)(void*);
float(btVoronoiSimplexSolver_maxVertex)(void*);
bool(btVoronoiSimplexSolver_inSimplex)(void*,const void*);
void(btRaycastVehicle_btVehicleTuning_setMaxSuspensionTravelCm)(void*,float);
void(btAABB_setMax)(void*,const void*);
void*(btVoronoiSimplexSolver_getSimplexPointsQ)(void*);
void*(btVoronoiSimplexSolver_getSimplexPointsP)(void*);
void*(btGjkPairDetector_new2)(const void*,const void*,int,int,float,float,void*,void*);
void(btCollisionWorld_AllHitsRayResultCallback_getRayFromWorld)(void*,void*);
bool(btSoftBody_cutLink)(void*,const void*,const void*,float);
bool(btVoronoiSimplexSolver_getNeedsUpdate)(void*);
void(btTriangleMesh_addTriangle2)(void*,const void*,const void*,const void*,bool);
void(btSoftBody_setSoftBodySolver)(void*,void*);
void(btContactSolverInfoData_setWarmstartingFactor)(void*,float);
void(btAngularLimit_test)(void*,float);
void(btSoftBody_defaultCollisionHandler2)(void*,void*);
void(btTranslationalLimitMotor2_getTargetVelocity)(void*,void*);
void(btDefaultCollisionConfiguration_setPlaneConvexMultipointIterations3)(void*,int,int);
void*(btPoint2PointConstraint_getSetting)(void*);
void(btSoftBody_Config_setKCHR)(void*,float);
bool(btVoronoiSimplexSolver_getCachedValidClosest)(void*);
void(btVoronoiSimplexSolver_getCachedV)(void*,void*);
void(btChunk_delete)(void*);
void(btMultiBody_setBaseWorldTransform)(void*,const void*);
void(btConvexShape_batchedUnitVectorGetSupportingVertexWithoutMargin)(void*,const void*,void*,int);
float*(btTypedConstraint_btConstraintInfo2_getLowerLimit)(void*);
void(btHinge2Constraint_getAxis1)(void*,void*);
void*(btBroadphasePair_getAlgorithm)(void*);
void(btCollisionWorld_setForceUpdateAllAabbs)(void*,bool);
void(btIndexedMesh_delete)(void*);
void*(btCollisionObject_getCollisionShape)(void*);
void(btBroadphaseInterface_aabbTest)(void*,const void*,const void*,void*);
void(btMultiBodySolverConstraint_setJacAindex)(void*,int);
void(btAlignedObjectArray_btPersistentManifoldPtr_delete)(void*);
void*(btMultimaterialTriangleMeshShape_new3)(void*,bool,const void*,const void*);
bool(btVoronoiSimplexSolver_fullSimplex)(void*);
bool(btVoronoiSimplexSolver_emptySimplex)(void*);
void(btVoronoiSimplexSolver_compute_points)(void*,void*,void*);
void(btSoftBody_updateClusters)(void*);
bool(btVoronoiSimplexSolver_closestPtPointTetrahedron)(void*,const void*,const void*,const void*,const void*,const void*,void*);
void(btDefaultCollisionConstructionInfo_setDefaultMaxCollisionAlgorithmPoolSize)(void*,int);
void(btUniversalConstraint_getAnchor2)(void*,void*);
void(btMultiBody_clearConstraintForces)(void*);
void(btMultiBody_setJointPos)(void*,int,float);
void(btDefaultMotionState_getCenterOfMassOffset)(void*,void*);
bool(btConeTwistConstraint_isPastSwingLimit)(void*);
void(btVoronoiSimplexSolver_backup_closest)(void*,void*);
void(btVoronoiSimplexSolver_addVertex)(void*,const void*,const void*,const void*);
void(btDynamicsWorld_setWorldUserInfo)(void*,void*);
void*(btVoronoiSimplexSolver_new)();
int(btGImpactBvh_getLeftNode)(void*,int);
bool(btDispatcherInfo_getUseEpa)(void*);
void(btCharacterControllerInterface_preStep)(void*,void*);
void(btAlignedObjectArray_btSoftBody_Anchor_resizeNoInitialize)(void*,int);
float(btSliderConstraint_getUpperAngLimit)(void*);
void(btSubSimplexClosestResult_setUsedVertices)(void*,const void);
void(btManifoldPoint_setLateralFrictionDir2)(void*,const void*);
void(btSubSimplexClosestResult_setDegenerate)(void*,bool);
void(btTranslationalLimitMotor2_getSpringDamping)(void*,void*);
void(btCompoundShape_getChildTransform)(void*,int,void*);
void(btSubSimplexClosestResult_setClosestPointOnSimplex)(void*,const void*);
void(btSubSimplexClosestResult_setBarycentricCoordinates5)(void*,float,float,float,float);
void(btSoftBody_Pose_getScl)(void*,void*);
void(btMultiBodySolverConstraint_getContactNormal2)(void*,void*);
void(btSubSimplexClosestResult_setBarycentricCoordinates2)(void*,float);
void(btTranslationalLimitMotor2_setServoTarget)(void*,const void*);
void(btSoftBody_Anchor_getLocal)(void*,void*);
void(btSubSimplexClosestResult_setBarycentricCoordinates)(void*);
void(btContactSolverInfoData_setMinimumSolverBatchSize)(void*,int);
int(btPolyhedralConvexShape_getNumEdges)(void*);
void*(btSubSimplexClosestResult_getUsedVertices)(void*);
void*(btDantzigSolver_new)();
void(btSubSimplexClosestResult_getClosestPointOnSimplex)(void*,void*);
float*(btSubSimplexClosestResult_getBarycentricCoords)(void*);
void*(btBox2dShape_new)(const void*);
void(btUsageBitfield_setUsedVertexD)(void*,bool);
void(btMultiBodyConstraint_internalSetAppliedImpulse)(void*,int,float);
void*(btPrimitiveTriangle_new)();
void(btMultiBody_processDeltaVeeMultiDof2)(void*);
void(btDbvtBroadphase_setPid)(void*,int);
void(btUsageBitfield_setUsedVertexC)(void*,bool);
const void*(btCollisionWorld_RayResultCallback_getCollisionObject)(void*);
void(btUsageBitfield_setUsedVertexB)(void*,bool);
void(btMultiBodySolverConstraint_setRelpos2CrossNormal)(void*,const void*);
void(btUsageBitfield_setUnused4)(void*,bool);
void*(btCollisionWorld_LocalConvexResult_getLocalShapeInfo)(void*);
void(btUsageBitfield_setUnused3)(void*,bool);
void(btUsageBitfield_setUnused2)(void*,bool);
void*(btWorldImporter_createGeneric6DofSpringConstraint)(void*,void*,void*,const void*,const void*,bool);
void(btConvexInternalShape_getImplicitShapeDimensions)(void*,void*);
bool(btBroadphaseProxy_isConcave)(int);
bool(btUsageBitfield_getUsedVertexD)(void*);
bool(btUsageBitfield_getUsedVertexC)(void*);
bool(btUsageBitfield_getUsedVertexB)(void*);
void*(btGeneric6DofSpring2Constraint_new)(void*,void*,const void*,const void*);
void(btMultiBodyDynamicsWorld_removeMultiBodyConstraint)(void*,void*);
int(btConeTwistConstraint_getFlags)(void*);
void(btConvexCast_CastResult_delete)(void*);
bool(btCollisionObject_isStaticObject)(void*);
bool(btUsageBitfield_getUnused4)(void*);
bool(btUsageBitfield_getUnused2)(void*);
void(btMultiBody_getBasePos)(void*,void*);
float(btRaycastVehicle_btVehicleTuning_getSuspensionCompression)(void*);
void(btBroadphaseInterface_getBroadphaseAabb)(void*,void*,void*);
bool(btUsageBitfield_getUnused1)(void*);
void*(btAlignedObjectArray_btSoftBody_Tetra_at)(void*,int);
void(btVehicleRaycaster_delete)(void*);
void(btConvexTriangleCallback_setManifoldPtr)(void*,void*);
void*(btTranslationalLimitMotor2_new2)(const void*);
void*(btVehicleRaycaster_castRay)(void*,const void*,const void*,void*);
float(btContactSolverInfoData_getTau)(void*);
float(btHingeConstraint_getHingeAngle)(void*,const void*,const void*);
void(btSoftBody_SContact_setWeights)(void*,const void*);
void(btVehicleRaycaster_btVehicleRaycasterResult_setHitPointInWorld)(void*,const void*);
void(btConvexShape_localGetSupportVertexWithoutMarginNonVirtual)(void*,const void*,void*);
void(btVehicleRaycaster_btVehicleRaycasterResult_setHitNormalInWorld)(void*,const void*);
float(btSoftBody_Link_getC2)(void*);
void(btMultiBody_useGlobalVelocities)(void*,bool);
void(btVehicleRaycaster_btVehicleRaycasterResult_setDistFraction)(void*,float);
bool(btCollisionShape_isConcave)(void*);
void(btVehicleRaycaster_btVehicleRaycasterResult_getHitPointInWorld)(void*,void*);
void*(btSoftBody_RayFromToCaster_getFace)(void*);
void(btConvex2dConvex2dAlgorithm_CreateFunc_setNumPerturbationIterations)(void*,int);
const void*(btDbvt_sStkNP_getNode)(void*);
float(btVehicleRaycaster_btVehicleRaycasterResult_getDistFraction)(void*);
void*(btVehicleRaycaster_btVehicleRaycasterResult_new)();
void*(btMultiBody_getLink)(void*,int);
void(btPolyhedralConvexShape_getPlane)(void*,void*,void*,int);
void(btUniversalConstraint_setUpperLimit)(void*,float,float);
void*(btBroadphaseRayCallbackWrapper_new)(void*);
void(btUniversalConstraint_setLowerLimit)(void*,float,float);
void(btGImpactBvh_setNodeBound)(void*,int,const void*);
void**(btDispatcher_getInternalManifoldPointer)(void*);
int(btMultibodyLink_getParent)(void*);
void*(btDefaultMotionState_new)();
void(btUniversalConstraint_getAxis2)(void*,void*);
void(btDbvtProxy_setStage)(void*,int);
bool(btRotationalLimitMotor2_getEnableMotor)(void*);
void*(btSoftBodyConcaveCollisionAlgorithm_new)(const void*,const void*,const void*,bool);
void*(btHingeAccumulatedAngleConstraint_new2)(void*,void*,const void*,const void*,const void*,const void*,bool);
void(btMultiBody_setupFixed)(void*,int,float,const void*,int,const void*,const void*,const void*);
float(btGeneric6DofConstraint_getRelativePivotPosition)(void*,int);
void*(btUniversalConstraint_new)(void*,void*,const void*,const void*,const void*);
void(btUnionFind_sortIslands)(void*);
void(btUnionFind_reset)(void*,int);
int(btMultiBodyConstraint_getIslandIdB)(void*);
float(btWheelInfoConstructionInfo_getMaxSuspensionTravelCm)(void*);
void(btDiscreteDynamicsWorld_setLatencyMotionStateInterpolation)(void*,bool);
int(btUnionFind_getNumElements)(void*);
int(btWorldImporter_getNumRigidBodies)(void*);
int(btUnionFind_find2)(void*,int);
float(btSoftBodyWorldInfo_getWater_density)(void*);
int(btUnionFind_find)(void*,int,int);
void(btManifoldResult_setBody1Wrap)(void*,const void*);
void*(btSoftBodyConcaveCollisionAlgorithm_SwappedCreateFunc_new)();
int(btAABB_plane_classify)(void*,const void*);
void(btElement_delete)(void*);
void(btSoftBody_appendNote6)(void*,const char*,const void*,const void*,void*);
void(btSoftBody_clusterCom)(void*,int,void*);
void(btMultiBodyConstraint_setPosition)(void*,int,float);
void(btElement_setSz)(void*,int);
void(btCollisionObject_setCollisionShape)(void*,void*);
void(btElement_setId)(void*,int);
void*(btBulletWorldImporter_new2)(void*);
int(btElement_getSz)(void*);
int(btElement_getId)(void*);
bool(btDbvt_update4)(void*,void*,void*,float);
void*(btElement_new)();
bool(btBulletWorldImporter_loadFileFromMemory)(void*,char*,int);
const void*(btBox2dShape_getNormals)(void*);
int(btCollisionObject_calculateSerializeBufferSize)(void*);
int(btDbvt_nearest)(const int*,const void*,float,int,int);
float(btUniformScalingShape_getUniformScalingFactor)(void*);
bool(btCollisionShape_isCompound)(void*);
void(btRigidBody_getInvInertiaTensorWorld)(void*,void*);
void(btSoftBody_addVelocity)(void*,const void*);
bool(btMultiBody_hasFixedBase)(void*);
float(btSoftBody_RayFromToCaster_rayFromToTriangle)(const void*,const void*,const void*,const void*,const void*,const void*);
void*(btUniformScalingShape_getChildShape)(void*);
void*(btUniformScalingShape_new)(void*,float);
void(btTriangleMeshShape_getLocalAabbMax)(void*,void*);
void(btMultiBody_setMaxCoordinateVelocity)(void*,float);
void(btSoftBody_Link_setBbending)(void*,int);
void*(btSphereShape_new)(float);
void(btAlignedObjectArray_btSoftBody_Node_resizeNoInitialize)(void*,int);
void(btMultiBodyDynamicsWorld_addMultiBodyConstraint)(void*,void*);
void*(btSphereTriangleCollisionAlgorithm_new)(void*,const void*,const void*,const void*,bool);
void*(btChunk_getOldPtr)(void*);
const void*(btQuantizedBvhTree_get_node_pointer)(void*);
float(btSoftBodyWorldInfo_getWater_offset)(void*);
void(btQuantizedBvh_quantize)(void*,unsigned short*,const void*,int);
bool(btCollisionObject_isStaticOrKinematicObject)(void*);
void(btSoftBody_AJoint_IControlWrapper_setWrapperData)(void*,void*);
void(btSoftBody_Body_applyVImpulse)(void*,const void*,const void*);
void(btDbvtBroadphase_benchmark)(void*);
void(btGImpactShapeInterface_postUpdate)(void*);
int(btStridingMeshInterface_calculateSerializeBufferSize)(void*);
bool(btGeneric6DofSpring2Constraint_isLimited)(void*,int);
float(btSoftBody_Config_getKLF)(void*);
void(btContactSolverInfoData_setErp)(void*,float);
float(btAngularLimit_getCorrection)(void*);
void(btMultiBody_getLinkTorque)(void*,int,void*);
void(btAngularLimit_fit)(void*,float*);
void(btCollisionWorld_LocalConvexResult_setHitPointLocal)(void*,const void*);
void(btTypedConstraint_setUserConstraintType)(void*,int);
void(btTypedConstraint_setUserConstraintPtr)(void*,void*);
void(btConeTwistConstraint_setLimit2)(void*,float,float,float);
unsigned char*(btDefaultSerializer_internalAlloc)(void*,size_t);
void(btConvexCast_CastResult_getNormal)(void*,void*);
float(btGImpactMeshShapePart_TrimeshPrimitiveManager_getMargin)(void*);
void(btTypedConstraint_setupSolverConstraint)(void*,void*,int,int,float);
void(btTypedConstraint_setParam2)(void*,int,float,int);
void(btTypedConstraint_setParam)(void*,int,float);
void(btTypedConstraint_setOverrideNumSolverIterations)(void*,int);
void(btTypedConstraint_setJointFeedback)(void*,void*);
void(btWheelInfo_setRotation)(void*,float);
float(btSliderConstraint_getSoftnessDirAng)(void*);
void(btTypedConstraint_setDbgDrawSize)(void*,float);
void*(btRigidBody_btRigidBodyConstructionInfo_new)(float,void*,void*);
void(btTypedConstraint_setBreakingImpulseThreshold)(void*,float);
int(btMultiBody_getCompanionId)(void*);
const char*(btTypedConstraint_serialize)(void*,void*,void*);
float(btSoftBody_SolverState_getUpdmrg)(void*);
void(btTypedConstraint_internalSetAppliedImpulse)(void*,float);
const char*(btMultiBody_getBaseName)(void*);
void(btWheelInfoConstructionInfo_setWheelsDampingCompression)(void*,float);
int(btTypedConstraint_getUserConstraintType)(void*);
void*(btTypedConstraint_getRigidBodyB)(void*);
float(btSliderConstraint_getMaxAngMotorForce)(void*);
void(btTranslationalLimitMotor_setStopCFM)(void*,const void*);
int(btGjkPairDetector_getCatchDegeneracies)(void*);
void(btMultiBodyDynamicsWorld_addMultiBody2)(void*,void*,short);
float(btTypedConstraint_getParam2)(void*,int,int);
void(btManifoldPoint_setIndex1)(void*,int);
float(btTypedConstraint_getParam)(void*,int);
void(btDbvt_IWriter_WriteNode)(void*,const void*,int,int,int,int);
const void*(btMultimaterialTriangleMeshShape_getMaterialProperties)(void*,int,int);
void*(btTypedConstraint_getJointFeedback)(void*);
void*(btGjkPairDetector_new)(const void*,const void*,void*,void*);
void(btTypedConstraint_getInfo2)(void*,void*);
void(btTypedConstraint_getInfo1)(void*,void*);
void(btMultibodyLink_setDVector)(void*,const void*);
void(btGjkPairDetector_getClosestPointsNonVirtual)(void*,const void*,void*,void*);
float(btCylinderShape_getRadius)(void*);
void*(btVoronoiSimplexSolver_getCachedBC)(void*);
void*(btCollisionWorld_LocalConvexResult_new)(const void*,void*,const void*,const void*,float);
bool(btCollisionWorld_ConvexResultCallbackWrapper_needsCollision)(void*,void*);
int(btTypedConstraint_getConstraintType)(void*);
void(btStridingMeshInterface_getLockedReadOnlyVertexIndexBase)(void*,const unsigned char**,int*,int*,int*,const unsigned char**,int*,int*,int*);
void(btJointFeedback_setAppliedForceBodyA)(void*,const void*);
float(btTypedConstraint_getBreakingImpulseThreshold)(void*);
float(btTypedConstraint_getAppliedImpulse)(void*);
void(btMultiBody_getLinkInertia)(void*,int,void*);
bool(btDispatcherInfo_getEnableSPU)(void*);
float(btRotationalLimitMotor_getHiLimit)(void*);
int(btTypedConstraint_calculateSerializeBufferSize)(void*);
void(btSoftBody_setTotalDensity)(void*,float);
void*(btSoftBody_Pose_getPos)(void*);
void(btTypedConstraint_buildJacobian)(void*);
bool*(btTranslationalLimitMotor_getEnableMotor)(void*);
void(btMultiBody_addJointTorqueMultiDof2)(void*,int,int,float);
void(btAABB_merge)(void*,const void*);
void*(btSoftBody_Tetra_getLeaf)(void*);
float(btSoftBody_Tetra_getRv)(void*);
float(btSoftBody_Material_getKLST)(void*);
void*(btConeShapeX_new)(float,float);
void(btRigidBody_getGravity)(void*,void*);
void(btTypedConstraint_btConstraintInfo2_delete)(void*);
void(btTypedConstraint_btConstraintInfo2_setUpperLimit)(void*,float*);
void(btTypedConstraint_btConstraintInfo2_setRowskip)(void*,int);
void(btTypedConstraint_btConstraintInfo2_setNumIterations)(void*,int);
void(btTypedConstraint_btConstraintInfo2_setLowerLimit)(void*,float*);
void*(btSoftBody_sRayCast_getBody)(void*);
void(btTypedConstraint_btConstraintInfo2_setJ2linearAxis)(void*,float*);
void(btMultiBody_setBaseVel)(void*,const void*);
void(btTypedConstraint_btConstraintInfo2_setJ2angularAxis)(void*,float*);
void(btTypedConstraint_btConstraintInfo2_setJ1linearAxis)(void*,float*);
void(btTypedConstraint_btConstraintInfo2_setJ1angularAxis)(void*,float*);
float(btCapsuleShape_getHalfHeight)(void*);
void(btTypedConstraint_btConstraintInfo2_setFps)(void*,float);
void*(btCollisionObject_getBroadphaseHandle)(void*);
void(btTypedConstraint_btConstraintInfo2_setErp)(void*,float);
void(btTypedConstraint_btConstraintInfo2_setDamping)(void*,float);
void*(btAlignedObjectArray_btIndexedMesh_at)(void*,int);
void*(btWorldImporter_createScaledTrangleMeshShape)(void*,void*,const void*);
void(btTypedConstraint_btConstraintInfo2_setCfm)(void*,float*);
float*(btTypedConstraint_btConstraintInfo2_getUpperLimit)(void*);
bool(btCollisionShape_isSoftBody)(void*);
void(btConvexPointCloudShape_getScaledPoint)(void*,int,void*);
float(btMultiBodyConstraint_getMaxAppliedImpulse)(void*);
void*(btSoftBody_getAnchors)(void*);
float(btRotationalLimitMotor2_getHiLimit)(void*);
void(btRotationalLimitMotor2_setEnableMotor)(void*,bool);
void(btMultiBody_setBasePos)(void*,const void*);
const float*(btMultiBody_getVelocityVector)(void*);
float*(btTypedConstraint_btConstraintInfo2_getJ2linearAxis)(void*);
float*(btTypedConstraint_btConstraintInfo2_getJ2angularAxis)(void*);
void(btConvexHullShape_getScaledPoint)(void*,int,void*);
void(btGImpactBvh_delete)(void*);
float*(btTypedConstraint_btConstraintInfo2_getJ1angularAxis)(void*);
void(btDispatcher_freeCollisionAlgorithm)(void*,void*);
float(btTypedConstraint_btConstraintInfo2_getFps)(void*);
bool(btQuantizedBvh_isQuantized)(void*);
unsigned int(btQuantizedBvh_calculateSerializeBufferSize)(void*);
void(btCollisionWorld_setDebugDrawer)(void*,void*);
void(btDynamicsWorld_clearForces)(void*);
void(btDbvt_sStkNN_delete)(void*);
void*(btTypedConstraint_btConstraintInfo2_new)();
void*(btDiscreteDynamicsWorld_getCollisionWorld)(void*);
void(btSoftBody_appendNote5)(void*,const char*,const void*,const void*);
void(btMultibodyLink_setFlags)(void*,int);
void(btManifoldPoint_setCombinedRestitution)(void*,float);
int(btCollisionObjectWrapper_getPartId)(void*);
void(btCharacterControllerInterface_setVelocityForTimeInterval)(void*,const void*,float);
void(btConvexCast_CastResult_DebugDraw)(void*,float);
void(btTypedConstraint_btConstraintInfo1_setNub)(void*,int);
void*(btWheelInfoConstructionInfo_new)();
int(btTypedConstraint_btConstraintInfo1_getNub)(void*);
void*(btTypedConstraint_btConstraintInfo1_new)();
void*(btBox2dBox2dCollisionAlgorithm_new)(const void*);
bool(btNNCGConstraintSolver_getOnlyForNoneContact)(void*);
void*(btSoftBody_Element_getTag)(void*);
void(btDefaultSoftBodySolver_copySoftBodyToVertexBuffer)(void*,const void*,void*);
void(btJointFeedback_setAppliedTorqueBodyA)(void*,const void*);
bool(btDbvtBroadphase_getDeferedcollide)(void*);
bool*(btTranslationalLimitMotor2_getSpringDampingLimited)(void*);
void(btQuantizedBvh_deSerializeFloat)(void*,void*);
void(btSoftBodyWorldInfo_setMaxDisplacement)(void*,float);
int(btQuantizedBvhNode_getEscapeIndex)(void*);
float(btRotationalLimitMotor2_getEquilibriumPoint)(void*);
void(btJointFeedback_getAppliedForceBodyA)(void*,void*);
void*(btJointFeedback_new)();
void(btMultiBody_worldPosToLocal)(void*,int,const void*,void*);
void*(btTetrahedronShapeEx_new)();
bool(btTriangleShapeEx_overlap_test_conservative)(void*,const void*);
void*(btWorldImporter_createConvexHullShape)(void*);
void*(btTriangleShapeEx_new3)(const void*);
void*(btCapsuleShapeZ_new)(float,float);
void*(btTriangleShapeEx_new)();
void*(btConvexTriangleCallback_new)(void*,const void*,const void*,bool);
void(btPrimitiveTriangle_delete)(void*);
void(btRigidBody_getAngularFactor)(void*,void*);
void(btPrimitiveTriangle_setPlane)(void*,const void*);
void(btPrimitiveTriangle_setMargin)(void*,float);
void(btPrimitiveTriangle_setDummy)(void*,float);
bool(btRigidBody_btRigidBodyConstructionInfo_getAdditionalDamping)(void*);
bool(btPrimitiveTriangle_overlap_test_conservative)(void*,const void*);
void*(btPrimitiveTriangle_getVertices)(void*);
void(btPrimitiveTriangle_getPlane)(void*,void*);
float(btPrimitiveTriangle_getMargin)(void*);
int(btSoftBody_rayTest2)(void*,const void*,const void*,float*,int*,int*,bool);
void(btTriangleCallback_delete)(void*);
void(btRigidBody_btRigidBodyConstructionInfo_setAngularSleepingThreshold)(void*,float);
void(btManifoldPoint_setContactMotion1)(void*,float);
void(btSoftBody_sCti_getNormal)(void*,void*);
void(btConvexTriangleCallback_clearCache)(void*);
float(btContactSolverInfoData_getGlobalCfm)(void*);
void(btCollisionObject_setCcdSweptSphereRadius)(void*,float);
int(btPrimitiveTriangle_clip_triangle)(void*,void*,void*);
void(btGearConstraint_setAxisA)(void*,void*);
void*(btCollisionWorld_AllHitsRayResultCallback_getHitFractions)(void*);
void(btPrimitiveTriangle_applyTransform)(void*,const void*);
void(btRaycastVehicle_setSteeringValue)(void*,float,int);
int(btCollisionDispatcher_getDispatcherFlags)(void*);
void(btPersistentManifold_replaceContactPoint)(void*,const void*,int);
void(btTranslationalLimitMotor2_setCurrentLimitErrorHi)(void*,const void*);
void(btDbvt_sStkCLN_setParent)(void*,void*);
void(btMultiBodySolverConstraint_setCfm)(void*,float);
void*(btCollisionWorld_ClosestRayResultCallback_new)(const void*,const void*);
void(btMinkowskiSumShape_setTransformB)(void*,const void*);
void*(btSoftBody_Body_getSoft)(void*);
void(btGImpactBvh_buildSet)(void*);
void*(btDbvt_sStkNPS_new)();
void(btRaycastVehicle_btVehicleTuning_setMaxSuspensionForce)(void*,float);
void(btMultiBody_setUseGyroTerm)(void*,bool);
void*(btDispatcher_getManifoldByIndexInternal)(void*,int);
void(btRigidBody_btRigidBodyConstructionInfo_setMotionState)(void*,void*);
void*(btTriangleShape_getVertices1)(void*);
void*(btDispatcher_findAlgorithm2)(void*,const void*,const void*,void*);
const float*(btTriangleShape_getVertexPtr)(void*,int);
float(btSoftBody_Config_getKPR)(void*);
void(btTriangleShape_getPlaneEquation)(void*,int,void*,void*);
void(btPointCollector_getPointInWorld)(void*,void*);
void(btTriangleShape_calcNormal)(void*,void*);
void(btHingeConstraint_setAxis)(void*,void*);
void*(btTriangleShape_new2)(const void*,const void*,const void*);
void*(btTriangleShape_new)();
void(btBox2dShape_getCentroid)(void*,void*);
void(btTriangleMeshShape_localGetSupportingVertexWithoutMargin)(void*,const void*,void*);
void(btGImpactCollisionAlgorithm_setPart0)(void*,int);
void(btCollisionObject_setContactProcessingThreshold)(void*,float);
bool(btSoftBody_checkLink)(void*,const void*,const void*);
void(btSoftBody_Anchor_setC2)(void*,float);
void(btCollisionShape_getBoundingSphere)(void*,void*,float*);
void(btDispatcherInfo_setUseEpa)(void*,bool);
void(btCollisionWorld_LocalConvexResult_setHitNormalLocal)(void*,const void*);
void(btGeneric6DofConstraint_calculateTransforms2)(void*);
int(btRigidBody_getFlags)(void*);
void*(btGImpactMeshShapePart_TrimeshPrimitiveManager_getMeshInterface)(void*);
void(btSoftBody_Joint_setErp)(void*,float);
int(btTriangleMesh_getNumTriangles)(void*);
int(btTriangleMesh_findOrAddVertex)(void*,const void*,bool);
float(btRigidBody_getLinearDamping)(void*);
void(btTriangleMesh_addTriangleIndices)(void*,int,int,int);
void*(btMultiBodyConstraintSolver_new)();
void*(btCollisionConfiguration_getCollisionAlgorithmPool)(void*);
float(btSoftBody_Link_getRl)(void*);
void(btGeneric6DofConstraint_updateRHS)(void*,float);
void(btCollisionWorld_LocalConvexResult_setHitCollisionObject)(void*,const void*);
int(btWorldImporter_getNumConstraints)(void*);
void*(btTriangleMesh_new2)(bool);
void*(btTriangleMesh_new)();
const void*(btBvhTree_get_node_pointer)(void*);
void(btConstraintSetting_setImpulseClamp)(void*,float);
int(btSoftBody_Config_getPiterations)(void*);
void(btTriangleInfoMap_setPlanarEpsilon)(void*,float);
void(btManifoldPoint_getPositionWorldOnB)(void*,void*);
void(btStridingMeshInterface_unLockVertexBase)(void*,int);
void*(btGImpactMeshShape_getMeshInterface)(void*);
int(btMaterialProperties_getTriangleMaterialStride)(void*);
void(btTriangleInfoMap_setMaxEdgeAngleThreshold)(void*,float);
void(btTriangleInfoMap_setEdgeDistanceThreshold)(void*,float);
void(btGImpactBvh_setPrimitiveManager)(void*,void*);
void*(btSoftBodyConcaveCollisionAlgorithm_CreateFunc_new)();
void(btTriangleInfoMap_setConvexEpsilon)(void*,float);
const char*(btTriangleInfoMap_serialize)(void*,void*,void*);
bool(btCollisionObject_checkCollideWith)(void*,const void*);
void(btAlignedObjectArray_btSoftBodyPtr_resizeNoInitialize)(void*,int);
void(btDiscreteCollisionDetectorInterface_Result_delete)(void*);
void(btRaycastVehicle_btVehicleTuning_setSuspensionDamping)(void*,float);
float(btTriangleInfoMap_getPlanarEpsilon)(void*);
float(btTriangleInfoMap_getEqualVertexThreshold)(void*);
float(btTriangleInfoMap_getConvexEpsilon)(void*);
void(btTriangleIndexVertexMaterialArray_getLockedMaterialBase2)(void*,unsigned char**,int*,int*,int*,unsigned char**,int*,int*,int*,int);
void(btRigidBody_translate)(void*,const void*);
void(btGeneric6DofSpring2Constraint_calculateTransforms)(void*,const void*,const void*);
void(btSliderConstraint_setTargetAngMotorVelocity)(void*,float);
void(btTriangleInfo_delete)(void*);
int(btWorldImporter_getNumCollisionShapes)(void*);
void(btTriangleInfo_setEdgeV2V0Angle)(void*,float);
void(btTriangleInfo_setEdgeV1V2Angle)(void*,float);
void(btShapeHull_delete)(void*);
int(btTriangleInfo_getFlags)(void*);
int(btAlignedObjectArray_btPersistentManifoldPtr_size)(void*);
void*(btManifoldPoint_new2)(const void*,const void*,const void*,float);
float(btTriangleInfo_getEdgeV2V0Angle)(void*);
void(btRigidBody_removeConstraintRef)(void*,void*);
float(btTriangleInfo_getEdgeV1V2Angle)(void*);
void(btContactSolverInfoData_delete)(void*);
void(btSoftBody_RContact_getC0)(void*,void*);
void(btMultiBody_setNumLinks)(void*,int);
void*(btGImpactCompoundShape_new)();
int(btContactSolverInfoData_getMinimumSolverBatchSize)(void*);
float(btTriangleInfo_getEdgeV0V1Angle)(void*);
void*(btTriangleInfo_new)();
void*(btBU_Simplex1to4_new3)(const void*,const void*);
void(btMultiBody_getRVector)(void*,int,void*);
void(btTriangleIndexVertexMaterialArray_getLockedReadOnlyMaterialBase2)(void*,const unsigned char**,int*,int*,int*,const unsigned char**,int*,int*,int*,int);
void*(btQuantizedBvh_getSubtreeInfoArray)(void*);
int(btTriangleInfoMap_calculateSerializeBufferSize)(void*);
bool(btAngularLimit_isLimit)(void*);
void(btTriangleIndexVertexMaterialArray_addMaterialProperties)(void*,const void*);
void(btStridingMeshInterface_InternalProcessAllTriangles)(void*,void*,const void*,const void*);
void*(btTriangleIndexVertexMaterialArray_new)();
void(btMaterialProperties_delete)(void*);
void*(btSoftBody_Body_getRigid)(void*);
void(btMaterialProperties_setTriangleType)(void*,int);
void(btOverlappingPairCache_cleanProxyFromPairs)(void*,void*,void*);
void(btPolarDecomposition_delete)(void*);
void(btBvhTree_setNodeBound)(void*,int,const void*);
void(btConvexCast_CastResult_setFraction)(void*,float);
void(btMaterialProperties_setTriangleMaterialsBase)(void*,const unsigned char*);
int(btAlignedObjectArray_btSoftBody_JointPtr_size)(void**);
void(btMultibodyLink_setAxisTop)(void*,int,float,float,float);
void(btMaterialProperties_setNumTriangles)(void*,int);
void(btMaterialProperties_setNumMaterials)(void*,int);
bool(btPoint2PointConstraint_getUseSolveConstraintObsolete)(void*);
float(btRaycastVehicle_btVehicleTuning_getFrictionSlip)(void*);
void(btMaterialProperties_setMaterialType)(void*,int);
void(btMaterialProperties_setMaterialStride)(void*,int);
void(btGeneric6DofConstraint_getCalculatedTransformB)(void*,void*);
void*(btIDebugDrawWrapper_new)(void*,void*,void*,void*,void*,void*,void*,void*,void*,void*,void*,void*,void*,void*,void*,void*);
void(btMaterialProperties_setMaterialBase)(void*,const unsigned char*);
int(btMaterialProperties_getTriangleType)(void*);
void*(btWorldImporter_getConstraintByIndex)(void*,int);
unsigned long(btSequentialImpulseConstraintSolver_getRandSeed)(void*);
void(btOverlapFilterCallback_delete)(void*);
int(btMaterialProperties_getMaterialStride)(void*);
bool(btBroadphaseProxy_isNonMoving)(int);
void*(btUnionFind_getElement)(void*,int);
void*(btMaterialProperties_new)();
void(btDbvt_sStkNN_setA)(void*,const void*);
float(btConeTwistConstraint_getMaxMotorImpulse)(void*);
void(btSoftBody_applyClusters)(void*,bool);
void*(btCollisionObject_getUserPointer)(void*);
void(btTriangleIndexVertexArray_addIndexedMesh2)(void*,const void*,int);
void(btTriangleIndexVertexArray_addIndexedMesh)(void*,const void*);
void(btRotationalLimitMotor2_setMaxMotorForce)(void*,float);
void(btRaycastVehicle_updateWheelTransformsWS)(void*,void*);
void*(btSoftBody_Config_getDsequence)(void*);
void(btIndexedMesh_setVertexStride)(void*,int);
int(btCollisionWorld_getNumCollisionObjects)(void*);
void(btMultiBodyDynamicsWorld_removeMultiBody)(void*,void*);
void(btIndexedMesh_setTriangleIndexStride)(void*,int);
void(btIndexedMesh_setTriangleIndexBase)(void*,const unsigned char*);
void(btIndexedMesh_setNumVertices)(void*,int);
void(btIndexedMesh_setNumTriangles)(void*,int);
float(btHingeConstraint_getMaxMotorImpulse)(void*);
bool(btHingeConstraint_hasLimit)(void*);
void(btDispatcher_releaseManifold)(void*,void*);
bool(btGImpactShapeInterface_needsRetrieveTetrahedrons)(void*);
const unsigned int*(btShapeHull_getIndexPointer)(void*);
void*(btTranslationalLimitMotor_new)();
void(btMultiBody_getLinkForce)(void*,int,void*);
const unsigned char*(btIndexedMesh_getVertexBase)(void*);
int(btIndexedMesh_getTriangleIndexStride)(void*);
const unsigned char*(btIndexedMesh_getTriangleIndexBase)(void*);
void(btGjkPairDetector_getCachedSeparatingAxis)(void*,void*);
void(btTranslationalLimitMotor2_setCurrentLimitError)(void*,const void*);
void(btRigidBody_predictIntegratedTransform)(void*,float,void*);
void(btCollisionObject_setFriction)(void*,float);
void(btGImpactShapeInterface_getChildAabb)(void*,int,const void*,void*,void*);
int(btIndexedMesh_getNumVertices)(void*);
bool*(btTranslationalLimitMotor2_getEnableSpring)(void*);
int(btIndexedMesh_getNumTriangles)(void*);
void(btGeneric6DofConstraint_calcAnchorPos)(void*);
int(btGImpactBvh_getNodeCount)(void*);
float(btConstraintSetting_getImpulseClamp)(void*);
int(btIndexedMesh_getIndexType)(void*);
void*(btIndexedMesh_new)();
int(btGjkPairDetector_getFixContactNormalDirection)(void*);
bool(btGImpactShapeInterface_hasBoxSet)(void*);
void*(btDispatcher_getNewManifold)(void*,const void*,const void*);
float(btContactSolverInfoData_getSor)(void*);
void*(btInternalTriangleIndexCallbackWrapper_new)(void*);
void(btPrimitiveTriangle_get_edge_plane)(void*,int,void*);
void(btBroadphasePair_setAlgorithm)(void*,void*);
void(btCollisionWorld_debugDrawObject)(void*,const void*,const void*,const void*);
int(btTriangleBuffer_getNumTriangles)(void*);
bool(btDispatcherInfo_getUseConvexConservativeDistanceUtil)(void*);
void(btTranslationalLimitMotor_setCurrentLimitError)(void*,const void*);
void(btTriangle_delete)(void*);
void(btTriangle_setVertex2)(void*,const void*);
void(btRigidBody_applyCentralForce)(void*,const void*);
void(btTriangle_setVertex1)(void*,const void*);
void(btTriangle_setVertex0)(void*,const void*);
bool(btHashedOverlappingPairCache_needsBroadphaseCollision)(void*,void*,void*);
void(btGhostObject_rayTest)(void*,const void*,const void*,void*);
void(btHeightfieldTerrainShape_setUseDiamondSubdivision)(void*);
void(btTriangle_setTriangleIndex)(void*,int);
bool(btQuantizedBvhTree_isLeafNode)(void*,int);
void(btTriangle_setPartId)(void*,int);
void(btSoftBodyHelpers_DrawNodeTree3)(void*,void*,int,int);
void(btSoftBody_AJoint_Specs_setAxis)(void*,const void*);
void(btPersistentManifold_delete)(void*);
int(btTriangle_getTriangleIndex)(void*);
int(btTriangle_getPartId)(void*);
void*(btAlignedObjectArray_btSoftBody_Link_at)(void*,int);
void(btMultiBody_delete)(void*);
void*(btSoftBody_getTetras)(void*);
void(btHeightfieldTerrainShape_setUseZigzagSubdivision)(void*);
void(btCollisionAlgorithmCreateFunc_delete)(void*);
float(btSoftBody_Config_getKDF)(void*);
void(btRigidBody_applyImpulse)(void*,const void*,const void*);
void(btConvexSeparatingDistanceUtil_updateSeparatingDistance)(void*,const void*,const void*);
void*(btCollisionWorld_RayResultCallbackWrapper_new)(void*,void*);
void(btConvexSeparatingDistanceUtil_initSeparatingDistance)(void*,const void*,float,const void*,const void*);
void*(btWorldImporter_createCylinderShapeZ)(void*,float,float);
void(btRigidBody_setFlags)(void*,int);
float(btRigidBody_computeAngularImpulseDenominator)(void*,const void*);
void(btSoftBody_Cluster_getFramexform)(void*,void*);
void(btSoftBody_Impulse_getDrift)(void*,void*);
void*(btGhostPairCallback_new)();
void(btTransformUtil_calculateVelocityQuaternion)(const void*,const void*,const void*,const void*,float,void*,void*);
float(btConeTwistConstraint_getBiasFactor)(void*);
void(btSoftBody_Body_activate)(void*);
void(btConvexCast_CastResult_setAllowedPenetration)(void*,float);
void(btManifoldPoint_setContactMotion2)(void*,float);
void(btStridingMeshInterface_getLockedVertexIndexBase2)(void*,unsigned char**,int*,int*,int*,unsigned char**,int*,int*,int*,int);
void(btSoftBody_Cluster_getLv)(void*,void*);
void(btSoftBody_Joint_setSdrift)(void*,const void*);
void(btBU_Simplex1to4_addVertex)(void*,const void*);
void*(btTranslationalLimitMotor_new2)(const void*);
void(btRigidBody_setGravity)(void*,const void*);
void*(btBU_Simplex1to4_new5)(const void*,const void*,const void*,const void*);
void*(btConvex2dConvex2dAlgorithm_new)(void*,const void*,const void*,const void*,void*,void*,int,int);
void*(btBU_Simplex1to4_new2)(const void*);
void(btSoftBody_Joint_Terminate)(void*,float);
void*(btSoftSoftCollisionAlgorithm_new2)(void*,const void*,const void*,const void*);
void(btStridingMeshInterface_delete)(void*);
void(btStridingMeshInterface_setScaling)(void*,const void*);
void(btDbvt_optimizeBottomUp)(void*);
void(btStridingMeshInterface_setPremadeAabb)(void*,const void*,const void*);
const char*(btStridingMeshInterface_serialize)(void*,void*,void*);
void(btSoftBody_Cluster_getLocii)(void*,void*);
void(btStridingMeshInterface_preallocateVertices)(void*,int);
bool(btBulletWorldImporter_convertAllObjects)(void*,void*);
void(btStridingMeshInterface_preallocateIndices)(void*,int);
void(btSliderConstraint_getAncorInB)(void*,void*);
int(btGImpactQuantizedBvh_getRightNode)(void*,int);
int(btDynamicsWorld_stepSimulation2)(void*,float,int);
bool(btStridingMeshInterface_hasPremadeAabb)(void*);
void(btStridingMeshInterface_getScaling)(void*,void*);
int(btStridingMeshInterface_getNumSubParts)(void*);
void(btMultibodyLink_setJointDamping)(void*,float);
void(btSoftBody_addVelocity2)(void*,const void*,int);
void(btTransformUtil_calculateDiffAxisAngleQuaternion)(const void*,const void*,void*,float*);
void(btStridingMeshInterface_getLockedVertexIndexBase)(void*,unsigned char**,int*,int*,int*,unsigned char**,int*,int*,int*);
void*(btConvexHullShape_new4)(const float*,int,int);
void*(btGeneric6DofConstraint_getRotationalLimitMotor)(void*,int);
void(btSoftBody_Element_delete)(void*);
void(btMultibodyLink_updateCacheMultiDof2)(void*,float*);
int(btPersistentManifold_getCacheEntry)(void*,const void*);
void(btStaticPlaneShape_getPlaneNormal)(void*,void*);
int(btGImpactMeshShape_getMeshPartCount)(void*);
float(btStaticPlaneShape_getPlaneConstant)(void*);
int(btTypedConstraint_btConstraintInfo2_getNumIterations)(void*);
void*(btSphereTriangleCollisionAlgorithm_new2)(const void*);
int(btPersistentManifold_getCompanionIdB)(void*);
void(btDiscreteDynamicsWorld_debugDrawConstraint)(void*,void*);
void(btSoftBody_Tetra_setC1)(void*,float);
void(btKinematicCharacterController_setUpAxis)(void*,int);
float(btSliderConstraint_getRestitutionDirLin)(void*);
bool(btSoftBody_Pose_getBvolume)(void*);
int(btSoftBody_Node_getBattach)(void*);
void(btSoftBody_appendNote3)(void*,const char*,const void*,void*);
void*(btSphereSphereCollisionAlgorithm_CreateFunc_new)();
void(btSoftBody_appendNote2)(void*,const char*,const void*,void*);
float(btHingeConstraint_getLowerLimit)(void*);
int(btGImpactMeshShapePart_TrimeshPrimitiveManager_getStride)(void*);
void(btCollisionWorld_updateAabbs)(void*);
float(btSphereBoxCollisionAlgorithm_getSpherePenetration)(void*,const void*,const void*,void*,void*);
int(btPrimitiveManagerBase_get_primitive_count)(void*);
int(btSoftBody_Material_getFlags)(void*);
int(btSoftBody_generateBendingConstraints)(void*,int);
void(btHingeConstraint_setLimit4)(void*,float,float,float,float,float);
void*(btSphereBoxCollisionAlgorithm_new)(void*,const void*,const void*,const void*,bool);
void(btConvexTriangleCallback_getAabbMin)(void*,void*);
void(btDbvtProxy_setLeaf)(void*,void*);
void*(btSphereBoxCollisionAlgorithm_CreateFunc_new)();
void(btSoftBody_Joint_Specs_delete)(void*);
float(btRotationalLimitMotor_getNormalCFM)(void*);
void(btSoftBody_appendTetra)(void*,int,void*);
void(btSparseSdf3_Reset)(void*);
void(btMinkowskiSumShape_setTransformA)(void*,const void*);
int(btSparseSdf3_RemoveReferences)(void*,void*);
bool(btPrimitiveManagerBase_is_trimesh)(void*);
float(btRigidBody_btRigidBodyConstructionInfo_getRollingFriction)(void*);
void(btSparseSdf3_Initialize2)(void*,int);
int(btSoftBody_Config_getCollisions)(void*);
void(btConeTwistConstraint_setLimit)(void*,int,float);
void(btSoftBody_initializeClusters)(void*);
void(btMultiBody_worldDirToLocal)(void*,int,const void*,void*);
void(btKinematicCharacterController_setJumpSpeed)(void*,float);
void*(btWorldImporter_getBvhByIndex)(void*,int);
int(btManifoldPoint_getPartId0)(void*);
void*(btSoftSoftCollisionAlgorithm_new)(const void*);
void*(btSoftSoftCollisionAlgorithm_CreateFunc_new)();
void(btSoftRigidDynamicsWorld_setDrawFlags)(void*,int);
void*(btSoftRigidDynamicsWorld_getWorldInfo)(void*);
void(btMultibodyLink_setCachedRotParentToThis)(void*,const void*);
bool(btConeTwistConstraint_isMaxMotorImpulseNormalized)(void*);
void(btAlignedObjectArray_btSoftBody_MaterialPtr_resizeNoInitialize)(void*,int);
float(btSoftBody_SolverState_getIsdt)(void*);
void(btCharacterControllerInterface_warp)(void*,const void*);
void(btRigidBody_setCenterOfMassTransform)(void*,const void*);
void(btCollisionWorld_removeCollisionObject)(void*,void*);
int(btSoftRigidDynamicsWorld_getDrawFlags)(void*);
bool(btHingeConstraint_getUseReferenceFrameA)(void*);
void*(btPairSet_new)();
void*(btGImpactCompoundShape_new2)(bool);
float(btSliderConstraint_getLinDepth)(void*);
void(btSoftRigidDynamicsWorld_addSoftBody2)(void*,void*,short);
void(btGeneric6DofSpring2Constraint_setServoTarget)(void*,int,float);
void*(btSoftRigidDynamicsWorld_getSoftBodyArray)(void*);
void*(btRaycastVehicle_btVehicleTuning_new)();
float(btManifoldPoint_getContactMotion1)(void*);
void*(btSoftRigidDynamicsWorld_new2)(void*,void*,void*,void*,void*);
void(btGImpactShapeInterface_getBulletTriangle)(void*,int,void*);
void*(btSoftRigidCollisionAlgorithm_new)(void*,const void*,const void*,const void*,bool);
void(btSoftBodySolverOutput_delete)(void*);
float(btSoftBody_Config_getKSSHR_CL)(void*);
void(btGeneric6DofSpring2Constraint_setServo)(void*,int,bool);
void(btAlignedObjectArray_btSoftBody_MaterialPtr_push_back)(void*,void*);
void(btMultiBody_stepPositionsMultiDof3)(void*,float,float*,float*);
void(btSoftBodySolver_setNumberOfVelocityIterations)(void*,int);
void(btSoftBodySolver_setNumberOfPositionIterations)(void*,int);
void(btSoftBodySolver_predictMotion)(void*,float);
void*(btWorldImporter_createCollisionObject)(void*,const void*,void*,const char*);
void(btGeneric6DofConstraint_getFrameOffsetB)(void*,void*);
int(btSoftBodySolver_getNumberOfVelocityIterations)(void*);
void*(btFace_new)();
void(btConeTwistConstraint_getAFrame)(void*,void*);
int(btSoftBodySolver_getNumberOfPositionIterations)(void*);
void(btSoftBodyHelpers_DrawClusterTree2)(void*,void*,int);
int(btRotationalLimitMotor_testLimitValue)(void*,float);
void(btRigidBody_applyForce)(void*,const void*,const void*);
bool(btSoftBodySolver_checkInitialized)(void*);
void*(btMultibodyLink_getAbsFrameLocVelocity)(void*);
int(btGjkPairDetector_getLastUsedMethod)(void*);
int(btMultiBody_getParent)(void*,int);
void(btTriangle_getVertex1)(void*,void*);
void(btDiscreteDynamicsWorld_setApplySpeculativeContactRestitution)(void*,bool);
void(btRotationalLimitMotor_setMaxLimitForce)(void*,float);
void(btSoftBodyHelpers_DrawNodeTree2)(void*,void*,int);
void(btSoftBodyHelpers_DrawNodeTree)(void*,void*);
void(btSoftBodyHelpers_DrawInfos)(void*,void*,bool,bool,bool);
const void*(btAxisSweep3_getOverlappingPairUserCallback)(void*);
void(btSoftBody_RContact_getC1)(void*,void*);
void(btSoftBodyHelpers_DrawFrame)(void*,void*);
void(btRotationalLimitMotor2_setServoTarget)(void*,float);
void(btSoftBodyHelpers_DrawFaceTree2)(void*,void*,int);
void(btManifoldPoint_setPositionWorldOnB)(void*,const void*);
void(btCollisionShape_delete)(void*);
void*(btCollisionConfiguration_getCollisionAlgorithmCreateFunc)(void*,int,int);
void(btCollisionShape_getLocalScaling)(void*,void*);
void(btSoftBodySolver_copyBackToSoftBodies2)(void*,bool);
void(btWheelInfo_getWheelDirectionCS)(void*,void*);
void(btGImpactMeshShapePart_TrimeshPrimitiveManager_setPart)(void*,int);
void(btSoftBodyHelpers_Draw2)(void*,void*,int);
float(btMultiBodySolverConstraint_getLowerLimit)(void*);
void*(btSoftBodyHelpers_CreatePatchUV)(void*,const void*,const void*,const void*,const void*,int,int,int,bool);
void(btConeTwistConstraint_setMotorTarget)(void*,const void*);
void(btCollisionObject_setUserIndex)(void*,int);
void(btUniversalConstraint_getAnchor)(void*,void*);
void(btTranslationalLimitMotor2_setStopERP)(void*,const void*);
int(btSoftBody_getTetraVertexNormalData)(void*,float*);
void(btOptimizedBvhNode_setSubPart)(void*,int);
void(btCollisionWorld_rayTestSingle)(const void*,const void*,void*,const void*,const void*,void*);
void(btBroadphaseProxy_delete)(void*);
int(btSoftBody_getFaceVertexNormalData2)(void*,float*,float*);
int(btSoftBody_getFaceVertexNormalData)(void*,float*);
void*(btCollisionWorld_getDispatcher)(void*);
void(btSoftBody_VSolve_Links)(void*,float);
void(btMultibodyLink_setMass)(void*,float);
void(btSoftBody_updateNormals)(void*);
void(btMultiBody_addBaseConstraintForce)(void*,const void*);
void(btAABB_increment_margin)(void*,float);
bool(btVoronoiSimplexSolver_closestPtPointTriangle)(void*,const void*,const void*,const void*,const void*,void*);
void*(btAlignedObjectArray_btPersistentManifoldPtr_new)();
void(btSoftBody_updateBounds)(void*);
void(btHingeConstraint_updateRHS)(void*,float);
void(btWheelInfo_RaycastInfo_getContactPointWS)(void*,void*);
bool*(btTranslationalLimitMotor2_getEnableMotor)(void*);
void(btSoftBody_PSolve_Anchors)(void*,float,float);
void(btContactSolverInfoData_setMaxGyroscopicForce)(void*,float);
void(btRaycastVehicle_setUserConstraintType)(void*,int);
void*(btConvexPointCloudShape_getUnscaledPoints)(void*);
void(btSoftBody_Config_setKVC)(void*,float);
void(btSoftBody_translate)(void*,const void*);
void(btSoftBody_transform)(void*,const void*);
void(btGeneric6DofConstraint_getCalculatedTransformA)(void*,void*);
void(btSoftBody_Link_setC2)(void*,float);
float(btWheelInfo_getSuspensionStiffness)(void*);
void(btSoftBody_solveCommonConstraints)(void**,int,int);
void(btSoftBody_solveClusters2)(void*,float);
float*(btMultiBody_getJointVelMultiDof)(void*,int);
void(btSoftBody_setVolumeMass)(void*,float);
void(btSoftBody_setVolumeDensity)(void*,float);
void(btSoftBody_setWindVelocity)(void*,const void*);
void(btSoftBody_setVelocity)(void*,const void*);
void(btMultiBody_getBaseVel)(void*,void*);
void(btSoftBody_scale)(void*,const void*);
void(btSoftBody_setTotalMass)(void*,float);
bool(btOverlapCallback_processOverlap)(void*,void*);
void*(btOptimizedBvh_new)();
void(btSoftBody_setTag)(void*,void*);
void(btSoftBody_setSolver)(void*,int);
void(btVoronoiSimplexSolver_getLastW)(void*,void*);
void(btSoftBody_setRestLengthScale)(void*,float);
void(btSoftBody_setPose)(void*,bool,bool);
void*(btGImpactMeshShape_new)(void*);
void(btSoftBody_setMass)(void*,int,float);
void(btTranslationalLimitMotor2_setBounce)(void*,const void*);
void(btSoftBody_setInitialWorldTransform)(void*,const void*);
void*(btDbvtProxy_getLeaf)(void*);
void(btSoftBody_setTotalMass2)(void*,float,bool);
void(btSoftBody_rotate)(void*,const void*);
bool(btDbvt_empty)(void*);
void(btCollisionObject_activate2)(void*,bool);
const void*(btDbvt_sStkNN_getA)(void*);
void(btSoftBody_resetLinkRestLengths)(void*);
int(btBvhTree_getRightNode)(void*,int);
void(btMultiBody_getBaseOmega)(void*,void*);
void(btMultibodyLink_getAppliedTorque)(void*,void*);
void(btSoftBody_releaseClusters)(void*);
void(btSoftBody_releaseCluster)(void*,int);
void(btMultiBody_addLinkTorque)(void*,int,const void*);
void(btSoftBody_refine)(void*,void*,float,bool);
float(btPrimitiveTriangle_getDummy)(void*);
float(btRigidBody_computeImpulseDenominator)(void*,const void*,const void*);
bool(btSoftBody_rayTest)(void*,const void*,const void*,void*);
void(btRotationalLimitMotor2_setMotorERP)(void*,float);
bool(btBvhTriangleMeshShape_getOwnsBvh)(void*);
void*(btSoftBody_Joint_Specs_new)();
void(btSoftBody_PSolve_SContacts)(void*,float,float);
void(btSoftBody_PSolve_RContacts)(void*,float,float);
void(btMultiBody_setupPlanar2)(void*,int,float,const void*,int,const void*,const void*,const void*,bool);
void(btSoftBody_SContact_setNode)(void*,void*);
void(btSoftBody_prepareClusters)(void*,int);
void(btSoftBody_Note_setText)(void*,const char*);
void(btDiscreteCollisionDetectorInterface_ClosestPointInput_setMaximumDistanceSquared)(void*,float);
void*(btAlignedObjectArray_btSoftBody_JointPtr_at)(void**,int);
float(btSliderConstraint_getDampingOrthoAng)(void*);
void(btSoftBody_integrateMotion)(void*);
const void*(btQuantizedBvhTree_get_node_pointer2)(void*,int);
int(btGImpactMeshShapePart_getPart)(void*);
void(btAxisSweep3_quantize)(void*,unsigned short*,const void*,int);
void(btSoftBody_initDefaults)(void*);
const void*(btBoxBoxDetector_getBox1)(void*);
void(btSoftBody_indicesToPointers)(void*);
float(btSoftBody_getVolume)(void*);
void(btSoftBody_Anchor_setNode)(void*,void*);
void*(btSoftBody_getUserIndexMapping)(void*);
float(btSoftBody_getTimeacc)(void*);
void*(btBvhTriangleMeshShape_new4)(void*,bool,const void*,const void*,bool);
void*(btGImpactMeshShapePart_new2)(void*,int);
float(btHingeConstraint_getMotorTargetVelosity)(void*);
void*(btSoftBody_getTag)(void*);
void(btSimulationIslandManager_buildIslands)(void*,void*,void*);
void(btCharacterControllerInterface_setWalkDirection)(void*,const void*);
void(btTranslationalLimitMotor2_getCurrentLinearDiff)(void*,void*);
void*(btSoftBody_getSoftBodySolver)(void*);
void*(btBox2dBox2dCollisionAlgorithm_CreateFunc_new)();
void*(btSoftBody_getScontacts)(void*);
void(btMultiBody_updateCollisionObjectWorldTransforms)(void*,void*,void*);
bool(btRotationalLimitMotor_isLimited)(void*);
float(btSoftBody_getRestLengthScale)(void*);
void*(btSoftBody_getRcontacts)(void*);
void*(btSoftBody_getPose)(void*);
void*(btPolarDecomposition_new)();
float(btSoftBody_Joint_getErp)(void*);
int(btMultiBodySolverConstraint_getOrgDofIndex)(void*);
void*(btSoftBody_getNdbvt)(void*);
void*(btSoftBody_getMaterials)(void*);
void(btSoftBodyHelpers_DrawClusterTree)(void*,void*);
void*(btSoftBody_getJoints)(void*);
float(btConstraintSolver_solveGroup)(void*,void**,int,void**,int,void**,int,const void*,void*,void*);
void(btMultibodyLink_setCachedRVector)(void*,const void*);
int(btCollisionObject_getUserIndex)(void*);
void(btSoftBody_getInitialWorldTransform)(void*,void*);
void*(btBox2dShape_new2)(float);
void*(btSoftBody_getFdbvt)(void*);
void*(btConvex2dShape_new)(void*);
btAlignedObjectArray_const_btCollisionObjectPtr*(btSoftBody_getCollisionDisabledObjects)(void*);
void(btDbvt_IWriter_delete)(void*);
const void*(btPersistentManifold_getBody1)(void*);
void(btQuantizedBvhTree_clearNodes)(void*);
void(btSoftBody_AJoint_Specs_setIcontrol)(void*,void*);
void*(btSoftBody_getBounds)(void*);
int(btTypedConstraint_btConstraintInfo2_getRowskip)(void*);
int(btSoftBody_generateClusters2)(void*,int,int);
int(btPoint2PointConstraint_getFlags)(void*);
int(btSoftBody_generateClusters)(void*,int);
void*(btMultiBodyPoint2Point_new)(void*,int,void*,const void*,const void*);
float(btMultiBody_getJointVel)(void*,int);
void*(btSoftBody_Link_new2)(void*);
int(btSoftBody_generateBendingConstraints2)(void*,int,void*);
void(btConeTwistConstraint_getInfo1NonVirtual)(void*,void*);
float(btCollisionShape_getMargin)(void*);
int(btSoftBody_CJoint_getLife)(void*);
void*(btDbvt_sStkCLN_getParent)(void*);
void(btAlignedObjectArray_btIndexedMesh_push_back)(void*,void*);
void(btConeTwistConstraint_setLimit5)(void*,float,float,float,float,float,float);
void(btGeneric6DofConstraint_getLinearUpperLimit)(void*,void*);
void(btSoftBody_appendAnchor6)(void*,int,void*,bool,float);
void(btMultiBody_localPosToWorld)(void*,int,const void*,void*);
bool(btSoftBody_cutLink2)(void*,int,int,float);
void(btManifoldPoint_setPositionWorldOnA)(void*,const void*);
void(btSoftBody_clusterVImpulse)(void*,const void*,const void*);
void*(btSoftBody_getSst)(void*);
void(btDynamicsWorld_removeAction)(void*,void*);
void(btSoftBody_clusterVAImpulse)(void*,const void*);
void(btPointCollector_setNormalOnBInWorld)(void*,const void*);
void*(btRaycastVehicle_getRigidBody)(void*);
void(btCharacterControllerInterface_reset)(void*,void*);
void(btMultiBody_useRK4Integration)(void*,bool);
void(btDbvt_delete)(void*);
void(btGeneric6DofConstraint_setUseSolveConstraintObsolete)(void*,bool);
void(btSoftBody_clusterDImpulse)(void*,const void*,const void*);
float*(btMultiBodyConstraint_jacobianB)(void*,int);
void*(btSoftBody_Pose_getWgh)(void*);
void(btSoftBody_clusterDCImpulse)(void*,const void*);
float(btSoftBody_Config_getKVC)(void*);
void(btSoftBody_clusterDAImpulse)(void*,const void*);
bool(btCollisionShape_isNonMoving)(void*);
const void*(btSoftBody_sCti_getColObj)(void*);
void(btCollisionObject_getAnisotropicFriction)(void*,void*);
int(btSoftBody_clusterCount)(void*);
int(btQuantizedBvhTree_getRightNode)(void*,int);
void(btSoftBody_clusterCom2)(const void*,void*);
float(btWheelInfo_getSuspensionRestLength)(void*);
void(btOverlappingPairCache_sortOverlappingPairs)(void*,void*);
void(btMultiBodyConstraint_updateJacobianSizes)(void*);
bool(btGImpactQuantizedBvh_hasHierarchy)(void*);
void(btAxisSweep3_unQuantize)(void*,void*,void*,void*);
void(btSoftBody_cleanupClusters)(void*);
void*(btSoftBody_AJoint_Specs_new)();
bool(btSoftBody_checkLink2)(void*,int,int);
void*(btSoftBody_Anchor_getNode)(void*);
bool(btSoftBody_checkFace)(void*,int,int,int);
bool(btBroadphaseProxy_isPolyhedral)(int);
void(btVoronoiSimplexSolver_setEqualVertexThreshold)(void*,float);
float(btManifoldPoint_getContactMotion2)(void*);
void(btSoftBody_appendTetra3)(void*,int,int,int,int,void*);
void(btGImpactMeshShapePart_TrimeshPrimitiveManager_setIndicestype)(void*,int);
void(btSoftBody_appendNote9)(void*,const char*,const void*,const void*,void*,void*,void*,void*);
void(btTypedConstraint_btConstraintInfo1_delete)(void*);
void(btSoftBody_appendNote4)(void*,const char*,const void*);
void*(btSphereSphereCollisionAlgorithm_new)(void*,const void*,const void*,const void*);
void(btManifoldPoint_setAppliedImpulseLateral2)(void*,float);
void(btSoftBodyWorldInfo_getWater_normal)(void*,void*);
void(btDispatcherInfo_setUseContinuous)(void*,bool);
bool(btConvexPolyhedron_testContainment)(void*);
void(btSoftBody_appendLink9)(void*,void*,void*,void*,bool);
void(btConvexPolyhedron_getME)(void*,void*);
void(btSoftBody_appendLink6)(void*,int,void*);
void(btSoftBody_appendLink5)(void*,int);
void(btSoftBody_appendLink4)(void*);
void(btSoftBody_Link_setRl)(void*,float);
void*(btBoxShape_new3)(float,float,float);
void(btSoftBody_appendLink2)(void*,int,int,void*);
bool(btDbvt_ICollide_AllLeaves)(void*,const void*);
void(btSoftBody_Node_getX)(void*,void*);
void(btConvexInternalAabbCachingShape_recalcLocalAabb)(void*);
void(btTriangleInfoMap_delete)(void*);
void(btSoftBodyWorldInfo_setGravity)(void*,const void*);
void(btDiscreteCollisionDetectorInterface_getClosestPoints)(void*,const void*,void*,void*);
void(btSoftBody_sRayCast_delete)(void*);
void(btSoftBody_appendLinearJoint3)(void*,const void*,void*);
void(btRigidBody_computeGyroscopicImpulseImplicit_Body)(void*,float,void*);
void(btSoftBody_updateArea2)(void*,bool);
void(btRigidBody_getTotalTorque)(void*,void*);
float(btRaycastVehicle_rayCast)(void*,void*);
void*(btMultiBody_new)(int,float,const void*,bool,bool);
void*(btMultimaterialTriangleMeshShape_new2)(void*,bool,bool);
void(btSoftBody_appendFace3)(void*,int,void*);
void(btSoftBody_appendFace)(void*);
int(btMLCPSolver_getNumFallbacks)(void*);
void(btSoftBody_Cluster_setMaxSelfCollisionImpulse)(void*,float);
void(btSoftBody_appendAngularJoint2)(void*,const void*,void*);
void(btWheelInfoConstructionInfo_getWheelDirectionCS)(void*,void*);
bool(btDispatcherInfo_getEnableSatConvex)(void*);
void(btSoftBody_dampClusters)(void*);
float*(btFace_getPlane)(void*);
void**(btDbvtProxy_getLinks)(void*);
void*(btDbvtNode_getParent)(void*);
float(btSoftBody_Joint_Specs_getCfm)(void*);
void(btSoftBody_appendAnchor2)(void*,int,void*,const void*,bool);
void(btSoftBody_appendAnchor)(void*,int,void*,const void*);
void(btRigidBody_getLinearFactor)(void*,void*);
void(btGeneric6DofSpring2Constraint_setRotationOrder)(void*,int);
const void*(btSoftBody_Body_getCollisionObject)(void*);
float(btCollisionWorld_LocalRayResult_getHitFraction)(void*);
void(btCollisionObject_setWorldTransform)(void*,const void*);
bool(btGImpactQuantizedBvh_boxQueryTrans)(void*,const void*,const void*,void*);
void(btBroadphaseProxy_setAabbMin)(void*,const void*);
void(btBoxShape_getHalfExtentsWithoutMargin)(void*,void*);
void*(btSoftBody_new)(void*,int,const float*,const float*);
void(btSoftBody_Tetra_setRv)(void*,float);
void(btSoftBody_Tetra_setLeaf)(void*,void*);
void(btSoftBody_Tetra_setC2)(void*,float);
void*(btSphereTriangleCollisionAlgorithm_CreateFunc_new)();
void(btSoftBody_Body_delete)(void*);
void**(btSoftBody_Tetra_getN)(void*);
void(btCollisionObjectWrapper_setParent)(void*,const void*);
float(btSoftBody_Tetra_getC1)(void*);
void*(btSoftBody_Tetra_getC0)(void*);
void(btSoftBody_sRayCast_setIndex)(void*,int);
void(btSoftBody_sRayCast_setFraction)(void*,float);
void(btSoftBody_sRayCast_setFeature)(void*,int);
void(btSoftBody_sRayCast_setBody)(void*,void*);
void(btCharacterControllerInterface_playerStep)(void*,void*,float);
void(btSoftBody_AJoint_IControl_delete)(void*);
int(btSoftBody_sRayCast_getIndex)(void*);
float(btSoftBody_sRayCast_getFraction)(void*);
void(btChunk_setOldPtr)(void*,void*);
void(btIDebugDraw_delete)(void*);
void*(btSoftBody_Joint_getRefs)(void*);
void(btMultiBodySolverConstraint_setFrictionIndex)(void*,int);
void(btSoftBody_SolverState_setVelmrg)(void*,float);
void(btSoftBody_SolverState_setUpdmrg)(void*,float);
void(btContactSolverInfoData_setSplitImpulsePenetrationThreshold)(void*,float);
void(btSoftBody_SolverState_setSdt)(void*,float);
void(btSoftBody_SolverState_setRadmrg)(void*,float);
void*(btGImpactMeshShape_getMeshPart)(void*,int);
void(btSoftBody_SolverState_setIsdt)(void*,float);
float(btSoftBody_SolverState_getVelmrg)(void*);
void(btRotationalLimitMotor_setCurrentPosition)(void*,float);
void(btSoftBody_Pose_getAqq)(void*,void*);
bool(btTypedConstraint_needsFeedback)(void*);
float(btSoftBody_SolverState_getSdt)(void*);
float(btSoftBody_SolverState_getRadmrg)(void*);
void(btSoftBody_sCti_delete)(void*);
void(btSoftBody_sCti_setOffset)(void*,float);
int(btMultiBodyDynamicsWorld_getNumMultiBodyConstraints)(void*);
int(btCapsuleShape_getUpAxis)(void*);
float(btContactSolverInfoData_getLinearSlop)(void*);
float(btManifoldResult_calculateCombinedFriction)(const void*,const void*);
void(btSoftBody_sCti_setNormal)(void*,const void*);
void*(btCompoundShape_getDynamicAabbTree)(void*);
void(btCollisionAlgorithmConstructionInfo_setDispatcher1)(void*,void*);
void(btSoftBody_sCti_setColObj)(void*,const void*);
void(btGhostObject_removeOverlappingObjectInternal)(void*,void*,void*);
void*(btConvexConvexAlgorithm_CreateFunc_getPdSolver)(void*);
float(btSoftBody_sCti_getOffset)(void*);
void*(btSoftBody_sCti_new)();
void(btDbvt_sStkCLN_delete)(void*);
void*(btSoftBody_Impulse_new)();
void(btVehicleRaycaster_btVehicleRaycasterResult_delete)(void*);
void(btSoftBody_SContact_setNormal)(void*,const void*);
bool(btDispatcher_needsCollision)(void*,const void*,const void*);
void(btSoftBody_PSolve_Links)(void*,float,float);
bool(btBroadphaseProxy_isCompound)(int);
void(btSoftBody_SContact_setMargin)(void*,float);
void(btSoftBody_SContact_setFriction)(void*,float);
void*(btCollisionWorld_ContactResultCallbackWrapper_new)(void*,void*);
void(btSoftBody_SContact_setFace)(void*,void*);
int(btConeTwistConstraint_getSolveSwingLimit)(void*);
void*(btMultiBodyConstraint_getMultiBodyA)(void*);
void*(btSoftBody_CJoint_getRpos)(void*);
void(btSoftBody_SContact_getNormal)(void*,void*);
void(btRotationalLimitMotor_setStopERP)(void*,float);
void*(btSoftBody_SContact_getNode)(void*);
float(btTriangleMesh_getWeldingThreshold)(void*);
float(btSoftBody_SContact_getFriction)(void*);
float*(btSoftBody_SContact_getCfm)(void*);
void(btMultiBody_addLinkConstraintForce)(void*,int,const void*);
void(btAxisSweep3_updateHandle)(void*,unsigned short,const void*,const void*,void*);
void(btSoftBody_Node_setQ)(void*,const void*);
void(btSoftBody_RContact_setNode)(void*,void*);
void(btHingeConstraint_setUseFrameOffset)(void*,bool);
void(btSoftBody_RContact_setC4)(void*,float);
void*(btSoftBody_AJoint_IControl_Default)();
void(btSoftBody_RContact_setC3)(void*,float);
void(btSoftBody_RContact_setC2)(void*,float);
void*(btCollisionWorld_getDebugDrawer)(void*);
void(btSoftBody_RContact_setC1)(void*,const void*);
void(btSoftBody_RContact_setC0)(void*,const void*);
bool(btPolyhedralConvexShape_initializePolyhedralFeatures2)(void*,int);
void(btMultibodyLink_getInertiaLocal)(void*,void*);
void*(btSoftBody_RContact_getNode)(void*);
float(btSoftBody_RContact_getC3)(void*);
void(btMultibodyLink_setAxisBottom2)(void*,int,const void*);
float(btSoftBody_RContact_getC2)(void*);
void(btGImpactCollisionAlgorithm_gimpact_vs_shape)(void*,const void*,const void*,const void*,const void*,bool);
void(btSoftBody_RayFromToCaster_setTests)(void*,int);
void(btSoftBody_RayFromToCaster_setRayNormalizedDirection)(void*,const void*);
void(btCollisionDispatcher_defaultNearCallback)(void*,void*,const void*);
void(btSoftBody_RayFromToCaster_setRayTo)(void*,const void*);
void*(btSoftBody_Face_getLeaf)(void*);
void(btSoftBody_RayFromToCaster_setRayFrom)(void*,const void*);
void(btSoftBody_RayFromToCaster_setMint)(void*,float);
float(btCollisionWorld_ConvexResultCallback_addSingleResult)(void*,void*,bool);
void(btMLCPSolver_setNumFallbacks)(void*,int);
bool(btTranslationalLimitMotor_isLimited)(void*,int);
void(btGjkPairDetector_setPenetrationDepthSolver)(void*,void*);
void(btSoftBody_RayFromToCaster_setFace)(void*,void*);
float(btSoftBody_RayFromToCaster_rayFromToTriangle2)(const void*,const void*,const void*,const void*,const void*,const void*,float);
int(btSoftBody_RayFromToCaster_getTests)(void*);
void(btSoftBody_RayFromToCaster_getRayTo)(void*,void*);
void(btDbvt_write)(void*,void*);
void(btPoint2PointConstraint_getPivotInA)(void*,void*);
float(btSoftBody_RayFromToCaster_getMint)(void*);
void(btSoftBody_Joint_Prepare)(void*,float,int);
void*(btBroadphasePair_new2)(const void*);
float(btConeTwistConstraint_getTwistAngle)(void*);
void(btCollisionWorld_ClosestConvexResultCallback_getConvexToWorld)(void*,void*);
float(btSliderConstraint_getDampingLimAng)(void*);
void(btSoftBody_Pose_setVolume)(void*,float);
void(btSoftBody_Pose_setScl)(void*,const void*);
void(btSoftBody_Pose_setRot)(void*,const void*);
void(btSliderConstraint_setSoftnessLimLin)(void*,float);
void(btSoftBody_Pose_setCom)(void*,const void*);
void(btSoftBody_Pose_setBvolume)(void*,bool);
void(btConeTwistConstraint_setLimit3)(void*,float,float,float,float);
void(btSoftBody_Pose_setBframe)(void*,bool);
void(btSoftBody_Pose_setAqq)(void*,const void*);
void(btDefaultMotionState_getStartWorldTrans)(void*,void*);
void(btSoftBody_Pose_getRot)(void*,void*);
void(btSoftBody_Pose_getCom)(void*,void*);
bool(btSoftBody_Pose_getBframe)(void*);
void(btSoftBody_predictMotion)(void*,float);
void(btSoftBody_Note_setRank)(void*,int);
void(btBvhTriangleMeshShape_refitTree)(void*,const void*,const void*);
void(btAABB_setMin)(void*,const void*);
void(btSoftBody_Note_setOffset)(void*,const void*);
const char*(btSoftBody_Note_getText)(void*);
float(btContactSolverInfoData_getErp2)(void*);
void*(btCylinderShape_new2)(float,float,float);
void*(btGeneric6DofConstraint_getTranslationalLimitMotor)(void*);
bool(btRotationalLimitMotor2_getServoMotor)(void*);
void(btSoftBody_Note_getOffset)(void*,void*);
void**(btSoftBody_Note_getNodes)(void*);
void(btGeneric6DofConstraint_setUseLinearReferenceFrameA)(void*,bool);
void*(btPairCachingGhostObject_new)();
void(btSoftBody_Node_setV)(void*,const void*);
void(btSoftBody_RContact_delete)(void*);
void*(btBox2dShape_new3)(float,float,float);
void(btSoftBody_Node_setLeaf)(void*,void*);
void(btSoftBody_Node_setIm)(void*,float);
void(btCollisionWorld_ClosestConvexResultCallback_getConvexFromWorld)(void*,void*);
void(btSoftBody_Node_setF)(void*,const void*);
void(btDiscreteDynamicsWorld_setNumTasks)(void*,int);
void(btActionInterface_debugDraw)(void*,void*);
bool(btDiscreteDynamicsWorld_getSynchronizeAllMotionStates)(void*);
void(btRigidBody_getAabb)(void*,void*,void*);
void(btSoftBody_Node_setBattach)(void*,int);
void(btSoftBody_Node_setArea)(void*,float);
float(btConvexCast_CastResult_getAllowedPenetration)(void*);
int(btContactSolverInfoData_getRestingContactRestitutionThreshold)(void*);
void(btSoftBody_Node_getV)(void*,void*);
void*(btQuantizedBvhTree_new)();
int(btMultiBodySolverConstraint_getFrictionIndex)(void*);
void*(btCompoundCollisionAlgorithm_CreateFunc_new)();
void(btSoftBody_Node_getQ)(void*,void*);
void*(btBroadphaseInterface_getOverlappingPairCache)(void*);
int(btMultibodyLink_getPosVarCount)(void*);
float(btAngularLimit_getSign)(void*);
void*(btSoftBody_appendMaterial)(void*);
void(btSoftBody_Node_getF)(void*,void*);
void(btCollisionObjectWrapper_getWorldTransform)(void*,void*);
void(btMultiBodyConstraint_debugDraw)(void*,void*);
void(btCompoundShape_updateChildTransform2)(void*,int,const void*,bool);
float(btSoftBody_Node_getArea)(void*);
void(btSoftBody_Material_setKVST)(void*,float);
void(btMultiBody_fillConstraintJacobianMultiDof)(void*,int,const void*,const void*,const void*,float*,void*,void*,void*);
const void*(btCollisionWorld_LocalRayResult_getCollisionObject)(void*);
void(btSoftBody_Material_setKLST)(void*,float);
void(btQuantizedBvh_setQuantizationValues2)(void*,const void*,const void*,float);
void(btGeneric6DofSpring2Constraint_calculateTransforms2)(void*);
void(btSoftBody_Material_setKAST)(void*,float);
void(btSoftBody_Material_setFlags)(void*,int);
float(btSoftBody_Material_getKVST)(void*);
bool(btSphereBoxCollisionAlgorithm_getSphereDistance)(void*,const void*,void*,void*,float*,const void*,float,float);
void*(btSoftBody_LJoint_getRpos)(void*);
void(btSoftBody_LJoint_Specs_setPosition)(void*,const void*);
void(btSoftBody_LJoint_Specs_getPosition)(void*,void*);
void*(btSoftBody_LJoint_Specs_new)();
int(btHingeConstraint_getFlags)(void*);
void(btSoftBody_appendLink3)(void*,int,int,void*,bool);
int(btGImpactBvh_getNodeData)(void*,int);
float(btSoftBody_Joint_Specs_getSplit)(void*);
void(btSoftBody_Link_setC3)(void*,const void*);
bool(btDbvtNode_isinternal)(void*);
void*(btDynamicsWorld_getWorldUserInfo)(void*);
void(btSoftBody_staticSolve)(void*,int);
void(btAlignedObjectArray_btVector3_push_back2)(void*,const void*);
void*(btWorldImporter_createPoint2PointConstraint2)(void*,void*,void*,const void*,const void*);
void*(btCollisionWorld_getDispatchInfo)(void*);
void(btAngularLimit_delete)(void*);
void**(btSoftBody_Link_getN)(void*);
void(btGeneric6DofConstraint_setUseFrameOffset)(void*,bool);
void*(btMultiBodyJointLimitConstraint_new)(void*,int,float,float);
void(btNodeOverlapCallback_processNode)(void*,int,int);
float(btSoftBody_Link_getC1)(void*);
void*(btWorldImporter_createRigidBody)(void*,bool,float,const void*,void*,const char*);
void*(btDbvtBroadphase_new2)(void*);
void(btTranslationalLimitMotor2_getLowerLimit)(void*,void*);
void(btDbvt_optimizeIncremental)(void*,int);
void*(btConeShapeZ_new)(float,float);
void*(btSoftBody_Link_new)();
void*(btBoxShape_new2)(float);
void(btGImpactCollisionAlgorithm_registerAlgorithm)(void*);
float(btDispatcherInfo_getTimeOfImpact)(void*);
void*(btConvexCast_CastResult_getDebugDrawer)(void*);
void(btSoftBody_Joint_delete)(void*);
void(btSoftBody_Joint_Type)(void*);
bool(btGeometryUtil_isPointInsidePlanes)(const void*,const void*,float);
void(btSoftBody_Joint_Solve)(void*,float,float);
void*(btConvexConcaveCollisionAlgorithm_CreateFunc_new)();
void(btSoftBody_Joint_setSplit)(void*,float);
int(btBvhTree_getEscapeNodeIndex)(void*,int);
void(btCollisionWorld_updateSingleAabb)(void*,void*);
void*(btDbvtNode_new)();
void(btBU_Simplex1to4_reset)(void*);
void(btSoftBody_Joint_setMassmatrix)(void*,const void*);
bool(btTriangleMesh_getUse32bitIndices)(void*);
void(btSoftBody_Joint_setDrift)(void*,const void*);
void(btOptimizedBvh_updateBvhNodes)(void*,void*,int,int,int);
void(btCollisionShape_getAnisotropicRollingFrictionDirection)(void*,void*);
int(btGImpactQuantizedBvh_getLeftNode)(void*,int);
void(btSoftBody_Joint_setDelete)(void*,bool);
void*(btHingeConstraint_new6)(void*,void*,const void*,const void*,bool);
void(btSoftBody_Joint_setCfm)(void*,float);
void(btPairSet_push_pair)(void*,int,int);
void*(btSoftBody_RayFromToCaster_new)(const void*,const void*,float);
void(btSoftBody_Body_xform)(void*,void*);
void(btMultibodyLink_getAppliedForce)(void*,void*);
float(btSoftBody_Joint_getSplit)(void*);
void(btSoftBody_Joint_getSdrift)(void*,void*);
void(btConvex2dConvex2dAlgorithm_CreateFunc_setMinimumPointsPerturbationThreshold)(void*,int);
void*(btGhostObject_new)();
float(btSoftBody_Cluster_getAdamping)(void*);
float(btSoftBody_Joint_getCfm)(void*);
void*(btSoftBody_Joint_getBodies)(void*);
void*(btBroadphasePair_getPProxy1)(void*);
void(btSoftBody_Joint_Specs_setSplit)(void*,float);
void(btRaycastVehicle_updateWheelTransformsWS2)(void*,void*,bool);
void(btSoftBody_Joint_Specs_setErp)(void*,float);
void(btMultibodyLink_setPosVarCount)(void*,int);
int(btGImpactMeshShapePart_TrimeshPrimitiveManager_getIndexstride)(void*);
int(btCollisionShape_getUserIndex)(void*);
float(btRigidBody_btRigidBodyConstructionInfo_getRestitution)(void*);
float(btSoftBody_Config_getKCHR)(void*);
void(btMultiBody_setCanSleep)(void*,bool);
int(btGhostObject_getNumOverlappingObjects)(void*);
void(btSoftBody_Joint_Specs_setCfm)(void*,float);
float(btSoftBody_Joint_Specs_getErp)(void*);
void(btConvexPolyhedron_getExtents)(void*,void*);
void(btSliderConstraint_setSoftnessDirLin)(void*,float);
void*(btConvexPolyhedron_getUniqueEdges)(void*);
void*(btConvexPointCloudShape_new2)(void*,int,const void*);
void(btSoftBody_appendAnchor3)(void*,int,void*,const void*,bool,float);
void(btRaycastVehicle_btVehicleTuning_delete)(void*);
const char*(btCollisionShape_getName)(void*);
void(btSoftBody_Impulse_setVelocity)(void*,const void*);
void(btSoftBody_Impulse_setDrift)(void*,const void*);
void*(btDynamicsWorld_getSolverInfo)(void*);
void(btSoftBody_Impulse_setAsVelocity)(void*,int);
void(btSoftBody_Impulse_setAsDrift)(void*,int);
void(btSoftBody_updateLinkConstants)(void*);
void*(btSoftBody_Cluster_getMasses)(void*);
float*(btTypedConstraint_btConstraintInfo2_getConstraintError)(void*);
void(btMultiBodyLinkCollider_setMultiBody)(void*,void*);
void*(btContinuousConvexCollision_new2)(const void*,const void*);
void(btSoftBody_SContact_delete)(void*);
void(btSoftBody_ImplicitFn_delete)(void*);
float(btSoftBody_ImplicitFn_Eval)(void*,const void*);
void(btBvhTree_build_tree)(void*,void*);
void(btSoftBody_Feature_setMaterial)(void*,void*);
void(btSoftBody_Face_setRa)(void*,float);
int(btBvhTree_getNodeCount)(void*);
void(btSoftBody_Face_setNormal)(void*,const void*);
void(btDbvt_setOpath)(void*,unsigned int);
void(btSoftBody_Face_setLeaf)(void*,void*);
float(btSoftBody_Face_getRa)(void*);
bool(btCharacterControllerInterface_canJump)(void*);
void(btSoftBody_Face_getNormal)(void*,void*);
void**(btSoftBody_Face_getN)(void*);
void(btSoftBody_Element_setTag)(void*,void*);
void(btSoftBody_Config_setViterations)(void*,int);
void(btSoftBody_Config_setTimescale)(void*,float);
void(btSoftBody_Config_setPiterations)(void*,int);
void*(btSoftBody_Cluster_getNodes)(void*);
void(btSoftBody_Config_setMaxvolume)(void*,float);
void(btMultiBody_setBaseCollider)(void*,void*);
void(btVoronoiSimplexSolver_setCachedValidClosest)(void*,bool);
void*(btSoftBody_upcast)(void*);
void(btSoftBody_Config_setKSSHR_CL)(void*,float);
void(btDbvt_IClone_delete)(void*);
void(btSoftBody_Config_setKSS_SPLT_CL)(void*,float);
void(btSoftBody_Config_setKSRHR_CL)(void*,float);
void(btSoftBody_Config_setKSR_SPLT_CL)(void*,float);
void(btDefaultCollisionConfiguration_setConvexConvexMultipointIterations)(void*);
void(btSoftBody_Config_setKSKHR_CL)(void*,float);
const void*(btDbvt_sStkNPS_getNode)(void*);
void(btManifoldPoint_getPositionWorldOnA)(void*,void*);
void(btSoftBody_Config_setKSK_SPLT_CL)(void*,float);
void(btSoftBody_Config_setKSHR)(void*,float);
void(btSoftBody_Config_setKPR)(void*,float);
void(btSoftBody_Config_setKMT)(void*,float);
void(btSoftBody_Config_setKLF)(void*,float);
void(btSoftBody_Config_setKKHR)(void*,float);
void(btSoftBody_Config_setKDP)(void*,float);
void(btSoftBody_Config_setKDG)(void*,float);
void(btSoftBody_Config_setKDF)(void*,float);
void*(btContactConstraint_getContactManifold)(void*);
float(btVoronoiSimplexSolver_getEqualVertexThreshold)(void*);
void*(btDbvtAabbMm_FromPoints2)(const void*,int);
void(btDispatcher_dispatchAllCollisionPairs)(void*,void*,const void*,void*);
void(btSoftBody_Config_setKAHR)(void*,float);
void(btSoftBody_Config_setDiterations)(void*,int);
void(btConvexShape_getPreferredPenetrationDirection)(void*,int,void*);
void(btRotationalLimitMotor2_setStopCFM)(void*,float);
void*(btDispatcherInfo_getDebugDraw)(void*);
void(btRigidBody_getAngularVelocity)(void*,void*);
void(btSoftBody_Config_setCollisions)(void*,int);
void*(btMultiBodySolverConstraint_getOriginalContactPoint)(void*);
void(btSoftBody_Config_setCiterations)(void*,int);
void(btSoftBody_Config_setAeromodel)(void*,int);
void*(btSoftBody_Config_getVsequence)(void*);
void(btManifoldPoint_setContactPointFlags)(void*,int);
void(btCollisionObject_setIgnoreCollisionCheck)(void*,const void*,bool);
int(btSoftBody_Config_getViterations)(void*);
float(btSliderConstraint_getAngDepth)(void*);
void*(btSoftBody_Config_getPsequence)(void*);
void(btGjkPairDetector_setCachedSeparatingAxis)(void*,const void*);
void(btTriangleInfoMap_setZeroAreaThreshold)(void*,float);
float(btDispatcherInfo_getAllowedCcdPenetration)(void*);
void(btVector3_array_set)(void*,int,const void*);
void(btDiscreteCollisionDetectorInterface_ClosestPointInput_setTransformA)(void*,const void*);
void(btSoftBodySolver_updateSoftBodies)(void*);
float(btSoftBody_Config_getKSS_SPLT_CL)(void*);
float(btSoftBody_Config_getKSRHR_CL)(void*);
void(btTranslationalLimitMotor_getUpperLimit)(void*,void*);
float(btSoftBody_Config_getKSR_SPLT_CL)(void*);
float(btContactSolverInfoData_getSplitImpulseTurnErp)(void*);
float(btSoftBody_Config_getKSKHR_CL)(void*);
float(btSoftBody_Config_getKSHR)(void*);
float(btSoftBody_Config_getKMT)(void*);
float(btAngularLimit_getHigh)(void*);
float(btSoftBody_Config_getKKHR)(void*);
void(btConvexShape_getAabbNonVirtual)(void*,const void*,void*,void*);
int(btAlignedObjectArray_btSoftBody_Node_size)(void*);
void*(btWorldImporter_getRigidBodyByName)(void*,const char*);
int(btSoftBody_Config_getDiterations)(void*);
void(btMultibodyLink_getDVector)(void*,void*);
void(btCompoundShape_updateChildTransform)(void*,int,const void*);
int(btDefaultCollisionConstructionInfo_getUseEpaPenetrationAlgorithm)(void*);
int(btDynamicsWorld_getWorldType)(void*);
void(btGeneric6DofSpring2Constraint_setDamping2)(void*,int,float,bool);
void(btMultiBody_checkMotionAndSleepIfRequired)(void*,float);
void(btSoftBody_Cluster_setSelfCollisionImpulseFactor)(void*,float);
float(btHingeAccumulatedAngleConstraint_getAccumulatedHingeAngle)(void*);
void(btSoftBody_Cluster_setNvimpulses)(void*,int);
void(btRaycastVehicle_setUserConstraintId)(void*,int);
void(btSoftBody_Cluster_setNdimpulses)(void*,int);
void(btAlignedObjectArray_btSoftBody_Node_push_back)(void*,void*);
float(btRaycastVehicle_btVehicleTuning_getSuspensionDamping)(void*);
void(btCollisionWorld_ConvexResultCallback_setCollisionFilterGroup)(void*,short);
void(btSoftBody_Cluster_setNdamping)(void*,float);
const void*(btGImpactShapeInterface_getLocalBox)(void*);
void(btSoftBody_appendAngularJoint4)(void*,const void*,void*,void*);
void(btConvexConvexAlgorithm_CreateFunc_setMinimumPointsPerturbationThreshold)(void*,int);
void(btSoftBody_Cluster_setMatching)(void*,float);
void(btSoftBody_Cluster_setLv)(void*,const void*);
void*(btTriangleMesh_new3)(bool,bool);
void(btGImpactMeshShapePart_getVertex)(void*,int,void*);
void(btConstraintSolver_prepareSolve)(void*,int,int);
void(btGImpactCompoundShape_addChildShape)(void*,const void*,void*);
void(btSoftBody_Cluster_setLeaf)(void*,void*);
void(btConvex2dConvex2dAlgorithm_setLowLevelOfDetail)(void*,bool);
void(btSliderConstraint_setRestitutionDirAng)(void*,float);
void(btSoftBody_Cluster_setLdamping)(void*,float);
void(btSoftBody_Cluster_setInvwi)(void*,const void*);
int(btMaterialProperties_getNumTriangles)(void*);
bool(btGImpactShapeInterface_needsRetrieveTriangles)(void*);
float(btWheelInfo_getWheelsDampingCompression)(void*);
void(btSoftBody_Cluster_setFramexform)(void*,const void*);
float(btMultiBody_getKineticEnergy)(void*);
void(btSoftBody_Cluster_setCom)(void*,const void*);
void(btSoftBody_Cluster_setCollide)(void*,bool);
bool(btBroadphaseProxy_isInfinite)(int);
void(btMultiBodyJointMotor_setVelocityTarget)(void*,float);
const char*(btMultibodyLink_getJointName)(void*);
void(btSoftBody_Cluster_setClusterIndex)(void*,int);
void(btSoftBody_Cluster_setAv)(void*,const void*);
void(btSoftBody_Cluster_setAdamping)(void*,float);
void(btRaycastVehicle_getChassisWorldTransform)(void*,void*);
void*(btSoftBody_Cluster_getVimpulses)(void*);
void(btManifoldPoint_setPartId1)(void*,int);
void*(btCollisionWorld_getPairCache)(void*);
int(btSoftBody_Cluster_getNvimpulses)(void*);
void(btTransformUtil_integrateTransform)(const void*,const void*,const void*,float,void*);
void(btSimulationIslandManager_storeIslandActivationState)(void*,void*);
float(btSoftBody_Cluster_getNdamping)(void*);
float(btSoftBody_Cluster_getMaxSelfCollisionImpulse)(void*);
float(btSoftBody_Cluster_getMatching)(void*);
void(btTransformUtil_calculateDiffAxisAngle)(const void*,const void*,void*,float*);
void(btCollisionWorld_AllHitsRayResultCallback_setRayFromWorld)(void*,const void*);
const void*(btGImpactBvh_get_node_pointer2)(void*,int);
void(btConeTwistConstraint_calcAngleInfo2)(void*,const void*,const void*,const void*,const void*);
float(btSoftBody_Cluster_getLdamping)(void*);
void(btSoftBody_Cluster_getInvwi)(void*,void*);
float(btSoftBody_Cluster_getImass)(void*);
int(btSoftBody_Cluster_getNdimpulses)(void*);
void(btGeneric6DofSpring2Constraint_setAxis)(void*,const void*,const void*);
void*(btSoftBody_Cluster_getDimpulses)(void*);
void(btMultiBody_addJointTorqueMultiDof)(void*,int,const float*);
void(btMultiBodyConstraint_finalizeMultiDof)(void*);
void(btMultibodyLink_setJointFriction)(void*,float);
void(btRigidBody_clearForces)(void*);
bool(btSoftBody_Cluster_getCollide)(void*);
void(btCollisionWorld_ConvexResultCallback_delete)(void*);
int(btSoftBody_Cluster_getClusterIndex)(void*);
void(btCollisionWorld_LocalRayResult_delete)(void*);
void(btSoftBody_Joint_getDrift)(void*,void*);
void(btSoftBody_CJoint_setNormal)(void*,const void*);
bool(btDbvtBroadphase_getReleasepaircache)(void*);
void(btSoftBody_CJoint_setMaxlife)(void*,int);
void(btTranslationalLimitMotor2_testLimitValue)(void*,int,float);
int(btPersistentManifold_getCompanionIdA)(void*);
void(btSoftBody_CJoint_setLife)(void*,int);
void(btSoftBody_CJoint_setFriction)(void*,float);
float(btRotationalLimitMotor_getStopCFM)(void*);
void(btSoftBody_CJoint_getNormal)(void*,void*);
void(btSliderConstraint_setPoweredAngMotor)(void*,bool);
void(btSoftBody_Body_velocity)(void*,const void*,void*);
void(btSoftBody_Body_setSoft)(void*,void*);
void(btSoftBody_Body_setRigid)(void*,void*);
int(btDbvt_getLkhd)(void*);
void(btSoftBody_Body_setCollisionObject)(void*,const void*);
bool(btGImpactBvh_isLeafNode)(void*,int);
const void*(btDbvt_sStkCLN_getNode)(void*);
void(btSoftBody_Body_invWorldInertia)(void*,void*);
int(btGImpactShapeInterface_getNumChildShapes)(void*);
float(btSoftBody_Body_invMass)(void*);
void(btSoftBody_addForce)(void*,const void*);
void*(btDbvt_IClone_new)();
const void*(btGImpactQuantizedBvh_get_node_pointer)(void*);
float(btAngularLimit_getRelaxationFactor)(void*);
void(btMultibodyLink_setAppliedForce)(void*,const void*);
void(btWorldImporter_deleteAllData)(void*);
void(btSoftBody_Body_applyImpulse)(void*,const void*,const void*);
void(btSoftBody_Body_applyDImpulse)(void*,const void*,const void*);
void(btSoftBody_Body_applyDCImpulse)(void*,const void*);
void*(btManifoldPoint_new)();
void(btSoftBody_Body_applyDAImpulse)(void*,const void*);
void(btHingeConstraint_setMaxMotorImpulse)(void*,float);
const char*(btWorldImporter_getNameForPointer)(void*,const void*);
void(btTranslationalLimitMotor2_getMotorERP)(void*,void*);
void*(btSoftBody_Body_new3)(void*);
void*(btSoftBody_Body_new2)(const void*);
float(btCollisionObject_getCcdSweptSphereRadius)(void*);
void(btAABB_getMin)(void*,void*);
void*(btSoftBody_Body_new)();
void(btSoftBody_getWindVelocity)(void*,void*);
void(btSoftBody_Anchor_setLocal)(void*,const void*);
void(btSoftBody_Anchor_setInfluence)(void*,float);
void*(btTriangleMeshShape_getMeshInterface)(void*);
float(btRigidBody_btRigidBodyConstructionInfo_getFriction)(void*);
void*(btWheelInfo_new)(void*);
int(btRigidBody_getFrictionSolverType)(void*);
void(btSoftBody_Anchor_setC0)(void*,const void*);
void(btSoftBody_Anchor_setBody)(void*,void*);
float(btSoftBody_Anchor_getC2)(void*);
void(btHingeConstraint_setUseReferenceFrameA)(void*,bool);
void(btConvexConcaveCollisionAlgorithm_clearCache)(void*);
void(btSoftBody_Anchor_getC1)(void*,void*);
void*(btGeneric6DofSpring2Constraint_getTranslationalLimitMotor)(void*);
void(btSoftBody_Anchor_getC0)(void*,void*);
void*(btBvhTriangleMeshShape_getTriangleInfoMap)(void*);
void*(btCompoundShape_getChildList)(void*);
void(btContactSolverInfoData_setNumIterations)(void*,int);
void(btBvhTriangleMeshShape_buildOptimizedBvh)(void*);
float(btPersistentManifold_getContactProcessingThreshold)(void*);
void*(btGeneric6DofSpringConstraint_new2)(void*,const void*,bool);
void(btSoftBody_AJoint_setIcontrol)(void*,void*);
float(btMultiBodySolverConstraint_getJacDiagABInv)(void*);
void*(btSoftBody_AJoint_getIcontrol)(void*);
void*(btSoftBody_AJoint_getAxis)(void*);
void*(btMultiBodySolverConstraint_getMultiBodyB)(void*);
bool(btSoftBody_getBUpdateRtCst)(void*);
void*(btEmptyShape_new)();
void(btTriangle_getVertex0)(void*,void*);
void*(btSoftBody_AJoint_Specs_getIcontrol)(void*);
void(btSoftBody_AJoint_Specs_getAxis)(void*,void*);
float(btHinge2Constraint_getAngle2)(void*);
void(btSoftBody_AJoint_IControl_Prepare)(void*,void*);
void(btSoftBody_Node_getN)(void*,void*);
int(btDbvtNode_getDataAsInt)(void*);
void*(btDefaultVehicleRaycaster_new)(void*);
void*(btSoftBody_AJoint_IControlWrapper_new)(void*,void*);
void(btSoftBodyWorldInfo_delete)(void*);
void(btSoftBodyWorldInfo_setWater_offset)(void*,float);
void(btSoftBodyWorldInfo_setWater_normal)(void*,const void*);
void*(btDbvtNode_getVolume)(void*);
void(btSoftBodyWorldInfo_setWater_density)(void*,float);
void(btOverlappingPairCallback_removeOverlappingPairsContainingProxy)(void*,void*,void*);
void(btJointFeedback_setAppliedForceBodyB)(void*,const void*);
void(btSoftBody_appendLinearJoint4)(void*,const void*,void*,void*);
void(btTranslationalLimitMotor_delete)(void*);
void(btCollisionShape_calculateLocalInertia)(void*,float,void*);
void*(btConvexConvexAlgorithm_CreateFunc_new)(void*,void*);
void(btSoftBodyWorldInfo_setDispatcher)(void*,void*);
void(btMultiBodyDynamicsWorld_forwardKinematics)(void*);
void*(btOverlappingPairCache_findPair)(void*,void*,void*);
void(btSoftBodyWorldInfo_setBroadphase)(void*,void*);
void(btSoftBodyWorldInfo_setAir_density)(void*,float);
void(btTriangleIndexVertexMaterialArray_getLockedMaterialBase)(void*,unsigned char**,int*,int*,int*,unsigned char**,int*,int*,int*);
float(btSoftBody_Node_getIm)(void*);
void*(btOverlappingPairCallback_removeOverlappingPair)(void*,void*,void*,void*);
void(btMultiBodySolverConstraint_setUpperLimit)(void*,float);
void(btDbvtBroadphase_setPrediction)(void*,float);
float(btSoftBodyWorldInfo_getMaxDisplacement)(void*);
void(btStorageResult_setNormalOnSurfaceB)(void*,const void*);
void(btSliderConstraint_calculateTransforms)(void*,const void*,const void*);
void(btSoftBodyWorldInfo_getGravity)(void*,void*);
void*(btSoftBodyWorldInfo_getDispatcher)(void*);
void(btCollisionAlgorithmConstructionInfo_setManifold)(void*,void*);
float(btSoftBodyWorldInfo_getAir_density)(void*);
void(btSoftBodyHelpers_DrawFaceTree)(void*,void*);
void(btSliderConstraint_testLinLimits)(void*);
void(btSliderConstraint_testAngLimits)(void*);
void(btGeneric6DofSpringConstraint_enableSpring)(void*,int,bool);
void(btMultibodyLink_setAppliedConstraintForce)(void*,const void*);
int(btCompoundShapeChild_getChildShapeType)(void*);
void(btConvexHullShape_addPoint)(void*,const void*);
void(btSliderConstraint_setUpperLinLimit)(void*,float);
void(btSliderConstraint_setUpperAngLimit)(void*,float);
void(btSliderConstraint_setTargetLinMotorVelocity)(void*,float);
void*(btTriangleInfoMap_new)();
void(btPersistentManifold_setContactBreakingThreshold)(void*,float);
void(btDbvtAabbMm_delete)(void*);
int(btManifoldPoint_getPartId1)(void*);
bool(btSliderConstraint_getUseFrameOffset)(void*);
void(btSliderConstraint_setSoftnessLimAng)(void*,float);
void(btSliderConstraint_setSoftnessDirAng)(void*,float);
float(btSliderConstraint_getTargetAngMotorVelocity)(void*);
float(btContactSolverInfoData_getRestitution)(void*);
int(btAlignedObjectArray_btSoftBody_Node_index_of)(void*,void*);
void*(btWorldImporter_getCollisionShapeByIndex)(void*,int);
void(btPersistentManifold_setCompanionIdB)(void*,int);
void(btMultiBodySolverConstraint_setJacDiagABInv)(void*,float);
bool(btGImpactBvh_rayQuery)(void*,const void*,const void*,void*);
void(btAlignedObjectArray_btVector3_at)(void*,int,void*);
float(btMultiBodySolverConstraint_getFriction)(void*);
int(btCompoundShape_getNumChildShapes)(void*);
void(btSliderConstraint_setPoweredLinMotor)(void*,bool);
float(btSoftBody_CJoint_getFriction)(void*);
void(btSliderConstraint_setMaxLinMotorForce)(void*,float);
void(btSliderConstraint_setMaxAngMotorForce)(void*,float);
void(btSliderConstraint_setLowerLinLimit)(void*,float);
void(btSliderConstraint_setLowerAngLimit)(void*,float);
float(btRotationalLimitMotor_getCurrentPosition)(void*);
int(btBox2dShape_getVertexCount)(void*);
void(btHingeConstraint_setLimit)(void*,float,float);
float(btSliderConstraint_getRestitutionOrthoLin)(void*);
void(btSliderConstraint_setDampingOrthoAng)(void*,float);
void(btCollisionAlgorithmConstructionInfo_delete)(void*);
void(btMultiBodyDynamicsWorld_addMultiBody3)(void*,void*,short,short);
void(btSliderConstraint_setDampingLimLin)(void*,float);
bool(btBvhTree_isLeafNode)(void*,int);
void(btSliderConstraint_setDampingLimAng)(void*,float);
void(btAlignedObjectArray_btSoftBody_JointPtr_push_back)(void**,void*);
void(btDbvtBroadphase_collide)(void*,void*);
float(btWheelInfo_getEngineForce)(void*);
void(btSliderConstraint_setDampingDirAng)(void*,float);
void(btSliderConstraint_setSoftnessOrthoAng)(void*,float);
void(btManifoldPoint_getLocalPointA)(void*,void*);
float(btSliderConstraint_getUpperLinLimit)(void*);
void(btSubSimplexClosestResult_delete)(void*);
void*(btAxisSweep3_new2)(const void*,const void*,unsigned short);
float(btGeneric6DofSpring2Constraint_getRelativePivotPosition)(void*,int);
float(btSliderConstraint_getTargetLinMotorVelocity)(void*);
int(btDbvtProxy_getStage)(void*);
bool(btSliderConstraint_getSolveLinLimit)(void*);
bool(btSliderConstraint_getSolveAngLimit)(void*);
void*(btSliderConstraint_new2)(void*,const void*,bool);
float(btSliderConstraint_getSoftnessLimLin)(void*);
float(btRotationalLimitMotor_getStopERP)(void*);
int(btRaycastVehicle_getForwardAxis)(void*);
void(btSliderConstraint_setDampingOrthoLin)(void*,float);
float(btSliderConstraint_getRestitutionOrthoAng)(void*);
void(btCollisionObject_forceActivationState)(void*,int);
void(btMotionState_getWorldTransform)(void*,void*);
float(btRigidBody_getAngularDamping)(void*);
int(btMultiBodySolverConstraint_getDeltaVelBindex)(void*);
void(btPrimitiveTriangle_buildTriPlane)(void*);
float(btSliderConstraint_getRestitutionDirAng)(void*);
bool(btSliderConstraint_getPoweredLinMotor)(void*);
bool(btSliderConstraint_getPoweredAngMotor)(void*);
void(btRigidBody_saveKinematicState)(void*,float);
float(btSliderConstraint_getMaxLinMotorForce)(void*);
float(btSliderConstraint_getLowerLinLimit)(void*);
float(btSliderConstraint_getLowerAngLimit)(void*);
float(btSliderConstraint_getDampingDirAng)(void*);
void(btSoftRigidDynamicsWorld_addSoftBody3)(void*,void*,short,short);
bool(btCollisionObject_checkCollideWithOverride)(void*,const void*);
void(btMultiBody_addBaseConstraintTorque)(void*,const void*);
void(btMotionState_delete)(void*);
void(btGeneric6DofConstraint_getAngularLowerLimit)(void*,void*);
void(btAlignedObjectArray_btSoftBodyPtr_push_back)(void*,void*);
void(btSliderConstraint_getInfo1NonVirtual)(void*,void*);
bool(btUsageBitfield_getUsedVertexA)(void*);
float(btSliderConstraint_getDampingOrthoLin)(void*);
void(btSoftBody_pointersToIndices)(void*);
float(btSliderConstraint_getDampingDirLin)(void*);
void(btRaycastVehicle_updateFriction)(void*,float);
float(btSliderConstraint_getLinearPos)(void*);
void(btSliderConstraint_getCalculatedTransformB)(void*,void*);
float(btSoftBody_Config_getTimescale)(void*);
void(btConvexPointCloudShape_setPoints3)(void*,void*,int,bool,const void*);
void(btSliderConstraint_getAncorInA)(void*,void*);
void(btSimulationIslandManager_delete)(void*);
float(btHingeConstraint_getUpperLimit)(void*);
void(btSimulationIslandManager_setSplitIslands)(void*,bool);
void(btGeneric6DofConstraint_getLinearLowerLimit)(void*,void*);
bool(btSimulationIslandManager_getSplitIslands)(void*);
void(btSimulationIslandManager_findUnions)(void*,void*,void*);
void*(btBoxBoxDetector_new)(const void*,const void*);
void(btSimulationIslandManager_buildAndProcessIslands)(void*,void*,void*,void*);
void*(btSimulationIslandManager_new)();
void(btSimulationIslandManager_IslandCallback_processIsland)(void*,void**,int,void**,int,int);
void(btCollisionObjectWrapper_setIndex)(void*,int);
void(btTriangleInfo_setEdgeV0V1Angle)(void*,float);
float(btCollisionObject_getRollingFriction)(void*);
void*(btGImpactQuantizedBvh_getGlobalBox)(void*);
void(btOptimizedBvhNode_delete)(void*);
void**(btDbvtBroadphase_getStageRoots)(void*);
int(btShapeHull_numIndices)(void*);
void(btDispatcherInfo_setEnableSatConvex)(void*,bool);
const void*(btShapeHull_getVertexPointer)(void*);
int(btCollisionWorld_LocalShapeInfo_getShapePart)(void*);
int(btIndexedMesh_getVertexStride)(void*);
bool(btShapeHull_buildHull)(void*,float);
void*(btShapeHull_new)(const void*);
void(btConeTwistConstraint_getFrameOffsetB)(void*,void*);
void(btQuantizedBvhTree_setNodeBound)(void*,int,const void*);
void*(btDbvtBroadphase_new)();
void(btDbvtNode_setData)(void*,void*);
void(btDefaultSerializer_writeHeader)(void*,unsigned char*);
bool(btGeneric6DofSpringConstraint_isSpringEnabled)(void*,int);
void*(btDefaultSerializer_new)();
bool(btCollisionObject_hasContactResponse)(void*);
void(btCollisionWorld_ClosestConvexResultCallback_getHitNormalWorld)(void*,void*);
void(btChunk_setNumber)(void*,int);
void(btPairSet_delete)(void*);
void*(btWheelInfo_RaycastInfo_new)();
int(btManifoldPoint_getContactPointFlags)(void*);
void(btMultiBody_clearForcesAndTorques)(void*);
float(btManifoldResult_calculateCombinedRestitution)(const void*,const void*);
float(btWheelInfo_getDeltaRotation)(void*);
void(btAngularLimit_set2)(void*,float,float,float);
bool(btGImpactQuantizedBvh_rayQuery)(void*,const void*,const void*,void*);
float(btDbvtBroadphase_getVelocityPrediction)(void*);
void*(btConvexConcaveCollisionAlgorithm_new)(const void*,const void*,const void*,bool);
int(btChunk_getDna_nr)(void*);
int(btChunk_getChunkCode)(void*);
void(btSequentialImpulseConstraintSolver_setRandSeed)(void*,unsigned long);
int(btMaterialProperties_getNumMaterials)(void*);
void(btRotationalLimitMotor_delete)(void*);
unsigned long(btSequentialImpulseConstraintSolver_btRand2)(void*);
void(btMultiBody_getWorldToBaseRot)(void*,void*);
void*(btAlignedObjectArray_btSoftBodyPtr_at)(void*,int);
void*(btSequentialImpulseConstraintSolver_new)();
bool(btRigidBody_wantsSleeping)(void*);
void(btGImpactMeshShapePart_TrimeshPrimitiveManager_get_indices)(void*,int,unsigned int*,unsigned int*,unsigned int*);
int(btGeneric6DofSpring2Constraint_getRotationOrder)(void*);
void(btRigidBody_updateInertiaTensor)(void*);
void(btRigidBody_updateDeactivation)(void*,float);
int(btMultiSphereShape_getSphereCount)(void*);
void(btRigidBody_setSleepingThresholds)(void*,float,float);
void(btManifoldPoint_setAppliedImpulse)(void*,float);
void(btPrimitiveManagerBase_delete)(void*);
bool(btMultiBodyConstraint_isUnilateral)(void*);
void(btTranslationalLimitMotor2_getStopCFM)(void*,void*);
void(btConvexConvexAlgorithm_setLowLevelOfDetail)(void*,bool);
void(btRigidBody_setMotionState)(void*,void*);
void(btRigidBody_setLinearFactor)(void*,const void*);
void(btRigidBody_setInvInertiaDiagLocal)(void*,const void*);
void(btRigidBody_setFrictionSolverType)(void*,int);
void*(btConvexSeparatingDistanceUtil_new)(float,float);
void(btCollisionDispatcher_setNearCallback)(void*,void*);
void(btBroadphaseAabbCallback_delete)(void*);
void(btConvexCast_CastResult_setHitPoint)(void*,const void*);
void(btMultiBody_setBaseOmega)(void*,const void*);
void*(btMultibodyLink_getAxes)(void*);
void(btManifoldPoint_setLocalPointA)(void*,const void*);
void(btRigidBody_setDamping)(void*,float,float);
void(btRigidBody_setContactSolverType)(void*,int);
void(btSoftRigidDynamicsWorld_addSoftBody)(void*,void*);
void(btRigidBody_setAngularVelocity)(void*,const void*);
void(btSoftBody_applyForces)(void*);
void*(btConvex2dConvex2dAlgorithm_CreateFunc_getPdSolver)(void*);
void(btConvexPolyhedron_setExtents)(void*,const void*);
void(btConvex2dConvex2dAlgorithm_CreateFunc_setSimplexSolver)(void*,void*);
bool(btRigidBody_isInWorld)(void*);
void(btWheelInfo_setEngineForce)(void*,float);
void(btRigidBody_getTotalForce)(void*,void*);
void(btDbvt_sStkNP_delete)(void*);
void(btCollisionWorld_LocalShapeInfo_delete)(void*);
void(btCollisionWorld_LocalShapeInfo_setTriangleIndex)(void*,int);
void(btSliderConstraint_setRestitutionOrthoAng)(void*,float);
int(btDbvtBroadphase_getFixedleft)(void*);
float(btRotationalLimitMotor_getAccumulatedImpulse)(void*);
unsigned int*(btBroadphaseRayCallback_getSigns)(void*);
void(btQuantizedBvh_deSerializeDouble)(void*,void*);
void(btSoftBody_clusterImpulse)(void*,const void*,const void*);
float(btRigidBody_getInvMass)(void*);
void(btGImpactQuantizedBvh_delete)(void*);
void*(btMultiBodyConstraint_getMultiBodyB)(void*);
void(btRigidBody_getInvInertiaDiagLocal)(void*,void*);
void(btMultiBodyDynamicsWorld_debugDrawMultiBodyConstraint)(void*,void*);
void*(btConvexHullShape_getUnscaledPoints)(void*);
float(btMultiBody_getJointTorque)(void*,int);
int(btQuantizedBvhNode_getPartId)(void*);
void*(btRigidBody_getConstraintRef)(void*,int);
void(btAlignedObjectArray_btCollisionObjectPtr_push_back)(void*,void*);
bool(btSoftBody_Cluster_getContainsAnchor)(void*);
void*(btRigidBody_getBroadphaseProxy)(void*);
float(btRigidBody_getAngularSleepingThreshold)(void*);
void*(btConeShape_new)(float,float);
void(btBvhTriangleMeshShape_performConvexcast)(void*,void*,const void*,const void*,const void*,const void*);
void(btConcaveShape_processAllTriangles)(void*,void*,const void*,const void*);
void(btCompoundShapeChild_delete)(void*);
void*(btScaledBvhTriangleMeshShape_getChildShape)(void*);
void(btRigidBody_applyTorque)(void*,const void*);
int(btGImpactMeshShapePart_TrimeshPrimitiveManager_getNumfaces)(void*);
void(btDbvtBroadphase_setCid)(void*,int);
void(btHingeConstraint_getInfo2NonVirtual)(void*,void*,const void*,const void*,const void*,const void*);
void(btConvexSeparatingDistanceUtil_delete)(void*);
void(btDbvtBroadphase_setNeedcleanup)(void*,bool);
void(btSoftBodySolver_copyBackToSoftBodies)(void*);
void(btMultiBody_setBaseMass)(void*,float);
void(btRigidBody_applyDamping)(void*,float);
void(btRigidBody_applyCentralImpulse)(void*,const void*);
void(btRigidBody_addConstraintRef)(void*,void*);
void(btRigidBody_btRigidBodyConstructionInfo_setStartWorldTransform)(void*,const void*);
float(btHingeConstraint_getLimitRelaxationFactor)(void*);
void(btRigidBody_btRigidBodyConstructionInfo_setRollingFriction)(void*,float);
void(btRigidBody_btRigidBodyConstructionInfo_setRestitution)(void*,float);
void(btGeneric6DofConstraint_setLimit)(void*,int,float,float);
void(btRigidBody_btRigidBodyConstructionInfo_setMass)(void*,float);
void(btWheelInfo_setRollInfluence)(void*,float);
void(btRigidBody_btRigidBodyConstructionInfo_setLinearSleepingThreshold)(void*,float);
void(btMultiBodyDynamicsWorld_clearMultiBodyConstraintForces)(void*);
bool(btDbvtAabbMm_Contain)(void*,const void*);
void(btRigidBody_btRigidBodyConstructionInfo_setLinearDamping)(void*,float);
void(btDbvt_clone2)(void*,void*,void*);
void(btRigidBody_btRigidBodyConstructionInfo_setFriction)(void*,float);
void(btRigidBody_btRigidBodyConstructionInfo_setCollisionShape)(void*,void*);
bool(btPrimitiveTriangle_find_triangle_collision_clip_method)(void*,void*,void*);
void(btRigidBody_btRigidBodyConstructionInfo_setAngularDamping)(void*,float);
void(btCollisionWorld_computeOverlappingPairs)(void*);
void(btCollisionDispatcher_setDispatcherFlags)(void*,int);
void(btRigidBody_btRigidBodyConstructionInfo_setAdditionalDampingFactor)(void*,float);
void*(btRotationalLimitMotor_new)();
void(btRigidBody_btRigidBodyConstructionInfo_setAdditionalDamping)(void*,bool);
void(btRigidBody_btRigidBodyConstructionInfo_setAdditionalAngularDampingThresholdSqr)(void*,float);
bool(btMultiBody_hasSelfCollision)(void*);
void*(btManifoldResult_new2)(const void*,const void*);
unsigned int(btPolarDecomposition_maxIterations)(void*);
void(btRigidBody_btRigidBodyConstructionInfo_setAdditionalAngularDampingFactor)(void*,float);
void(btRigidBody_btRigidBodyConstructionInfo_getStartWorldTransform)(void*,void*);
void(btTranslationalLimitMotor_getMaxMotorForce)(void*,void*);
void(btSparseSdf3_Initialize3)(void*);
void(btDefaultCollisionConstructionInfo_setCollisionAlgorithmPool)(void*,void*);
float(btRigidBody_btRigidBodyConstructionInfo_getMass)(void*);
const void*(btBox2dShape_getVertices)(void*);
void(btRigidBody_btRigidBodyConstructionInfo_getLocalInertia)(void*,void*);
void*(btCollisionAlgorithmConstructionInfo_getManifold)(void*);
float(btRigidBody_btRigidBodyConstructionInfo_getLinearDamping)(void*);
void(btDbvt_setLkhd)(void*,int);
void*(btRigidBody_btRigidBodyConstructionInfo_getCollisionShape)(void*);
void(btConvexCast_CastResult_getHitTransformB)(void*,void*);
void(btDbvt_sStkNN_setB)(void*,const void*);
float(btRigidBody_btRigidBodyConstructionInfo_getAngularDamping)(void*);
void(btGjkPairDetector_setMinkowskiB)(void*,const void*);
int(btQuantizedBvhTree_getNodeCount)(void*);
float(btRigidBody_btRigidBodyConstructionInfo_getAdditionalDampingFactor)(void*);
void(btMultiBody_addLinkConstraintTorque)(void*,int,const void*);
void(btIndexedMesh_setVertexType)(void*,int);
void(btAlignedObjectArray_btPersistentManifoldPtr_push_back)(void*,void*);
void(btRaycastVehicle_updateWheelTransform2)(void*,int,bool);
void(btRaycastVehicle_updateWheelTransform)(void*,int);
void(btRaycastVehicle_updateVehicle)(void*,float);
void(btRaycastVehicle_updateSuspension)(void*,float);
void(btSoftBody_updateArea)(void*);
void(btActionInterface_updateAction)(void*,void*,float);
void(btConvexCast_CastResult_setHitTransformB)(void*,const void*);
void(btManifoldPoint_setLifeTime)(void*,int);
void(btKinematicCharacterController_setUseGhostSweepTest)(void*,bool);
void(btMultibodyLink_setDofOffset)(void*,int);
void(btGImpactShapeInterface_processAllTrianglesRay)(void*,void*,const void*,const void*);
void(btStorageResult_getNormalOnSurfaceB)(void*,void*);
void(btSoftBody_appendLinearJoint)(void*,const void*,void*);
void*(btCollisionWorld_new)(void*,void*,void*);
const void*(btCollisionObjectWrapper_getCollisionShape)(void*);
void(btCompoundShape_removeChildShapeByIndex)(void*,int);
bool(btPersistentManifold_validContactDistance)(void*,const void*);
void*(btRaycastVehicle_getWheelInfo2)(void*);
void*(btRaycastVehicle_getWheelInfo)(void*,int);
bool(btBroadphaseProxy_isConvex)(int);
int(btRaycastVehicle_getUserConstraintType)(void*);
void(btPoint2PointConstraint_updateRHS)(void*,float);
float(btGeneric6DofSpringConstraint_getDamping)(void*,int);
int(btRaycastVehicle_getRightAxis)(void*);
int(btPersistentManifold_getNumContacts)(void*);
void*(btDiscreteCollisionDetectorInterface_ClosestPointInput_new)();
int(btRaycastVehicle_getNumWheels)(void*);
void(btGearConstraint_setRatio)(void*,float);
void(btMultibodyLink_updateCacheMultiDof)(void*);
void*(btRaycastVehicle_new)(const void*,void*,void*);
void(btRaycastVehicle_btVehicleTuning_setSuspensionStiffness)(void*,float);
float(btTriangleInfoMap_getZeroAreaThreshold)(void*);
void(btRaycastVehicle_btVehicleTuning_setSuspensionCompression)(void*,float);
void*(btVoronoiSimplexSolver_getSimplexVectorW)(void*);
void(btRaycastVehicle_btVehicleTuning_setFrictionSlip)(void*,float);
float(btRigidBody_btRigidBodyConstructionInfo_getLinearSleepingThreshold)(void*);
float(btRaycastVehicle_btVehicleTuning_getMaxSuspensionTravelCm)(void*);
void*(btDefaultMotionState_new3)(const void*,const void*);
float(btRaycastVehicle_btVehicleTuning_getMaxSuspensionForce)(void*);
void(btQuantizedBvh_unQuantize)(void*,const unsigned short*,void*);
void(btQuantizedBvh_setTraversalMode)(void*,int);
int(btGImpactCollisionAlgorithm_getFace0)(void*);
const char*(btQuantizedBvh_serialize2)(void*,void*,void*);
bool(btQuantizedBvh_serialize)(void*,void*,unsigned int,bool);
void(btQuantizedBvh_reportBoxCastOverlappingNodex)(void*,void*,const void*,const void*,const void*,const void*);
void(btQuantizedBvh_reportAabbOverlappingNodex)(void*,void*,const void*,const void*);
float(btConeShape_getRadius)(void*);
void*(btAABB_new2)(const void*,const void*,const void*);
float(btAngularLimit_getSoftness)(void*);
void(btTriangleIndexVertexMaterialArray_getLockedReadOnlyMaterialBase)(void*,const unsigned char**,int*,int*,int*,const unsigned char**,int*,int*,int*);
void*(btAxisSweep3_new3)(const void*,const void*,unsigned short,void*);
void*(btQuantizedBvh_getQuantizedNodeArray)(void*);
bool(btGImpactBvh_hasHierarchy)(void*);
void(btMultiBody_computeAccelerationsArticulatedBodyAlgorithmMultiDof2)(void*,float,void*,void*,void*,bool);
bool(btUnionFind_isRoot)(void*,int);
void*(btQuantizedBvh_deSerializeInPlace)(void*,unsigned int,bool);
void*(btWorldImporter_createGimpactShape)(void*,void*);
float(btMultiBodySolverConstraint_getRhs)(void*);
void(btNodeOverlapCallback_delete)(void*);
void(btMultibodyLink_setAxisBottom)(void*,int,float,float,float);
void(btManifoldPoint_setLocalPointB)(void*,const void*);
void(btDynamicsWorld_synchronizeMotionStates)(void*);
void(btMultiBody_setJointVelMultiDof)(void*,int,float*);
void(btSoftBody_Link_getC3)(void*,void*);
void(btCollisionWorld_RayResultCallback_setClosestHitFraction)(void*,float);
int(btShapeHull_numVertices)(void*);
void(btOptimizedBvhNode_setTriangleIndex)(void*,int);
int(btQuantizedBvhTree_getNodeData)(void*,int);
int(btSoftBody_getTetraVertexData)(void*,float*);
void(btOptimizedBvhNode_setEscapeIndex)(void*,int);
void(btOptimizedBvhNode_setAabbMinOrg)(void*,const void*);
float*(btMultibodyLink_getJointPos)(void*);
void(btOptimizedBvhNode_setAabbMaxOrg)(void*,const void*);
float(btMultiBodyConstraint_getAppliedImpulse)(void*,int);
int(btOptimizedBvhNode_getEscapeIndex)(void*);
void(btDiscreteDynamicsWorld_applyGravity)(void*);
void(btDbvt_clear)(void*);
void*(btBroadphasePair_new3)(void*,void*);
void(btOptimizedBvhNode_getAabbMinOrg)(void*,void*);
int(btAlignedObjectArray_btIndexedMesh_size)(void*);
void*(btHingeConstraint_new)(void*,void*,const void*,const void*,const void*,const void*);
void(btTranslationalLimitMotor_setStopERP)(void*,const void*);
void*(btOptimizedBvhNode_new)();
void(btQuantizedBvhNode_delete)(void*);
void(btQuantizedBvhNode_setEscapeIndexOrTriangleIndex)(void*,int);
void(btConvexInternalShape_getLocalScalingNV)(void*,void*);
bool(btQuantizedBvhNode_isLeafNode)(void*);
void(btGeneric6DofSpring2Constraint_setAngularUpperLimit)(void*,const void*);
int(btQuantizedBvhNode_getTriangleIndex)(void*);
unsigned short*(btQuantizedBvhNode_getQuantizedAabbMin)(void*);
void(btBroadphaseInterface_rayTest)(void*,const void*,const void*,void*);
void*(btBoxBoxCollisionAlgorithm_CreateFunc_new)();
int(btRigidBody_getContactSolverType)(void*);
void(btJointFeedback_getAppliedTorqueBodyA)(void*,void*);
void*(btQuantizedBvhNode_new)();
void(btPolyhedralConvexAabbCachingShape_getNonvirtualAabb)(void*,const void*,void*,void*,float);
bool(btPolyhedralConvexShape_isInside)(void*,const void*,float);
float(btRotationalLimitMotor2_getMotorCFM)(void*);
void(btOptimizedBvh_refit)(void*,void*,const void*,const void*);
int(btMultiBody_calculateSerializeBufferSize)(void*);
void*(btNullPairCache_new)();
float(btTypedConstraint_getDbgDrawSize)(void*);
void(btCharacterControllerInterface_jump)(void*);
float(btMultiBodySolverConstraint_getCfm)(void*);
int(btPolyhedralConvexShape_getNumVertices)(void*);
void(btSubSimplexClosestResult_reset)(void*);
void(btGhostObject_addOverlappingObjectInternal)(void*,void*);
void(btPolyhedralConvexShape_getEdge)(void*,int,void*,void*);
void*(btAlignedObjectArray_btSoftBody_Anchor_at)(void*,int);
void(btMaterialProperties_setTriangleMaterialStride)(void*,int);
void(btMultiBody_computeAccelerationsArticulatedBodyAlgorithmMultiDof)(void*,float,void*,void*,void*);
void(btGeneric6DofSpring2Constraint_setDamping)(void*,int,float);
unsigned int(btPolarDecomposition_decompose)(void*,const void*,void*,void*);
void(btGImpactQuantizedBvh_getNodeTriangle)(void*,int,void*);
void*(btPolarDecomposition_new3)(float,unsigned int);
int(btDbvtBroadphase_getPid)(void*);
void(btGeneric6DofSpring2Constraint_getAngularLowerLimitReversed)(void*,void*);
void*(btBox2dBox2dCollisionAlgorithm_new2)(void*,const void*,const void*,const void*);
void(btCollisionWorld_objectQuerySingleInternal)(const void*,const void*,const void*,const void*,void*,float);
void(btPointCollector_setHasResult)(void*,bool);
void(btPointCollector_setDistance)(void*,float);
void*(btMultiSphereShape_new2)(const void*,const float*,int);
void(btTranslationalLimitMotor_getCurrentLimitError)(void*,void*);
void(btPointCollector_getNormalOnBInWorld)(void*,void*);
bool(btPointCollector_getHasResult)(void*);
float(btPointCollector_getDistance)(void*);
void*(btPointCollector_new)();
void(btGeneric6DofSpring2Constraint_getFrameOffsetA)(void*,void*);
void*(btBvhTree_new)();
int(btRaycastVehicle_getUserConstraintId)(void*);
void*(btConvexPolyhedron_getFaces)(void*);
int(btRigidBody_getNumConstraintRefs)(void*);
void(btMultiBodyDynamicsWorld_addMultiBody)(void*,void*);
void(btPoint2PointConstraint_getPivotInB)(void*,void*);
const void*(btGImpactShapeInterface_getPrimitiveManager)(void*);
void(btSoftBody_RayFromToCaster_getRayFrom)(void*,void*);
void(btQuantizedBvhTree_build_tree)(void*,void*);
void(btOverlapCallback_delete)(void*);
void(btPoint2PointConstraint_getInfo2NonVirtual)(void*,void*,const void*,const void*);
void(btCollisionWorld_RayResultCallback_delete)(void*);
void(btPoint2PointConstraint_getInfo1NonVirtual)(void*,void*);
void(btConeTwistConstraint_enableMotor)(void*,bool);
void*(btPoint2PointConstraint_new)(void*,void*,const void*,const void*);
bool(btDbvt_ICollide_Descent)(void*,const void*);
void(btConstraintSetting_setTau)(void*,float);
void(btSoftBody_appendLink)(void*,int,int);
void(btConstraintSetting_setDamping)(void*,float);
void(btChunk_setChunkCode)(void*,int);
float(btSliderConstraint_getRestitutionLimAng)(void*);
bool(btMultiBody_isAwake)(void*);
void*(btConstraintSetting_new)();
void*(btGImpactCollisionAlgorithm_CreateFunc_new)();
void(btMultiBody_finalizeMultiDof)(void*);
void*(btMultiBodyDynamicsWorld_getMultiBody)(void*,int);
int(btDbvtAabbMm_Classify)(void*,const void*,float,int);
void(btMultiBody_setupRevolute)(void*,int,float,const void*,int,const void*,const void*,const void*,const void*);
bool(btHingeConstraint_getAngularOnly)(void*);
void(btGImpactMeshShapePart_TrimeshPrimitiveManager_setMeshInterface)(void*,void*);
void(btConeTwistConstraint_getFrameOffsetA)(void*,void*);
void(btPersistentManifold_setNumContacts)(void*,int);
void(btWorldImporter_setDynamicsWorldInfo)(void*,const void*,const void*);
void(btPersistentManifold_setContactProcessingThreshold)(void*,float);
void(btSliderConstraint_setSoftnessOrthoLin)(void*,float);
void*(btCollisionWorld_getCollisionObjectArray)(void*);
void*(btRigidBody_getMotionState)(void*);
void(btPersistentManifold_setCompanionIdA)(void*,int);
void(btDynamicsWorld_setConstraintSolver)(void*,void*);
void(btSoftBody_updateConstants)(void*);
void(btTranslationalLimitMotor_setNormalCFM)(void*,const void*);
void*(btSoftBody_Anchor_getBody)(void*);
void*(btPersistentManifold_getContactPoint)(void*,int);
void(btCollisionAlgorithm_processCollision)(void*,const void*,const void*,const void*,void*);
void*(btSoftBody_getCfg)(void*);
const void*(btPersistentManifold_getBody0)(void*);
void(btPersistentManifold_clearUserCache)(void*,void*);
void(btPersistentManifold_clearManifold)(void*);
float(btRotationalLimitMotor2_getSpringDamping)(void*);
void*(btPersistentManifold_new)();
void*(btOverlappingPairCallback_addOverlappingPair)(void*,void*,void*);
void(btConvexCast_CastResult_reportFailure)(void*,int,int);
int(btGImpactCollisionAlgorithm_getPart1)(void*);
void(btDispatcherInfo_setEnableSPU)(void*,bool);
float*(btMultibodyLink_getJointTorque)(void*);
void(btHingeAccumulatedAngleConstraint_setAccumulatedHingeAngle)(void*,float);
void(btGeneric6DofSpring2Constraint_setMaxMotorForce)(void*,int,float);
void*(btSortedOverlappingPairCache_new)();
void*(btHashedOverlappingPairCache_getOverlapFilterCallback)(void*);
int(btHashedOverlappingPairCache_GetCount)(void*);
void(btBox2dShape_getHalfExtentsWithoutMargin)(void*,void*);
void*(btHashedOverlappingPairCache_new)();
void(btOverlappingPairCache_setOverlapFilterCallback)(void*,void*);
void(btOverlappingPairCache_processAllOverlappingPairs)(void*,void*,void*);
void(btHingeConstraint_getInfo1NonVirtual)(void*,void*);
void*(btGeneric6DofSpring2Constraint_new2)(void*,void*,const void*,const void*,int);
int(btOverlappingPairCache_getNumOverlappingPairs)(void*);
void(btOverlappingPairCache_cleanOverlappingPair)(void*,void*,void*);
int(btCollisionShape_calculateSerializeBufferSize)(void*);
void*(btGImpactMeshShapePart_new)();
void(btConvexShape_project)(void*,const void*,const void*,float*,float*,void*,void*);
float(btHingeConstraint_getLimitSoftness)(void*);
bool(btOptimizedBvh_serializeInPlace)(void*,void*,unsigned int,bool);
float(btRotationalLimitMotor2_getSpringStiffness)(void*);
bool(btPolyhedralConvexShape_initializePolyhedralFeatures)(void*);
void(btDbvtAabbMm_tMaxs)(void*,void*);
void*(btHingeConstraint_new3)(void*,const void*,const void*);
void*(btAxisSweep3_getHandle)(void*,unsigned short);
void(btBroadphaseInterface_getAabb)(void*,void*,void*,void*);
void*(btOptimizedBvh_deSerializeInPlace)(void*,unsigned int,bool);
void(btJointFeedback_setAppliedTorqueBodyB)(void*,const void*);
bool(btGeometryUtil_areVerticesBehindPlane)(const void*,const void*,float);
void*(btNNCGConstraintSolver_new)();
void(btHingeConstraint_setFrames)(void*,const void*,const void*);
void(btMultiSphereShape_getSpherePosition)(void*,int,void*);
void*(btRigidBody_upcast)(void*);
void*(btRotationalLimitMotor2_new2)(const void*);
void*(btSoftBodyHelpers_CreateFromConvexHull2)(void*,const float*,int,bool);
int(btTypedConstraint_getOverrideNumSolverIterations)(void*);
void*(btMultimaterialTriangleMeshShape_new4)(void*,bool,const void*,const void*,bool);
void*(btMultimaterialTriangleMeshShape_new)(void*,bool);
void*(btWorldImporter_getTriangleInfoMapByIndex)(void*,int);
void*(btSoftBodyWorldInfo_getSparsesdf)(void*);
void(btCollisionWorld_AllHitsRayResultCallback_getRayToWorld)(void*,void*);
void*(btConvexPointCloudShape_new3)(void*,int,const void*,bool);
void(btMultiBodySolverConstraint_setUnusedPadding4)(void*,float);
void(btMultiBodySolverConstraint_setSolverBodyIdB)(void*,int);
void(btWheelInfo_RaycastInfo_getWheelAxleWS)(void*,void*);
void(btMultiBodySolverConstraint_setRhsPenetration)(void*,float);
void(btMultiBodySolverConstraint_setRhs)(void*,float);
void(btUsageBitfield_setUsedVertexA)(void*,bool);
void(btContactSolverInfoData_setSplitImpulseTurnErp)(void*,float);
void(btWheelInfo_setWheelAxleCS)(void*,const void*);
void(btMultiBodySolverConstraint_setOverrideNumSolverIterations)(void*,int);
void(btMultiBodySolverConstraint_setOriginalContactPoint)(void*,void*);
void*(btManifoldResult_new)();
void(btMultiBodySolverConstraint_setOrgDofIndex)(void*,int);
void*(btCollisionWorld_LocalRayResult_getLocalShapeInfo)(void*);
void(btAlignedObjectArray_btBroadphasePair_resizeNoInitialize)(void*,int);
void(btMultiBodySolverConstraint_setOrgConstraint)(void*,void*);
void(btGImpactCollisionAlgorithm_gimpact_vs_gimpact)(void*,const void*,const void*,const void*,const void*);
void(btMultiBodySolverConstraint_setMultiBodyB)(void*,void*);
void(btDbvt_sStkNPS_setValue)(void*,float);
void(btMultiBodySolverConstraint_setMultiBodyA)(void*,void*);
void(btMultiBodySolverConstraint_setLinkB)(void*,int);
float(btWheelInfo_getMaxSuspensionForce)(void*);
void(btSliderConstraint_setRestitutionLimAng)(void*,float);
void(btCollisionWorld_serialize)(void*,void*);
bool(btBvhTriangleMeshShape_usesQuantizedAabbCompression)(void*);
void(btMultiBodySolverConstraint_setJacBindex)(void*,int);
bool(btGImpactBvh_boxQueryTrans)(void*,const void*,const void*,void*);
void(btPolyhedralConvexShape_getVertex)(void*,int,void*);
void*(btSoftBody_sRayCast_new)();
int(btAlignedObjectArray_btSoftBody_Anchor_size)(void*);
void(btCylinderShape_getHalfExtentsWithoutMargin)(void*,void*);
void*(btSoftBody_SContact_new)();
void(btMultiBodySolverConstraint_setDeltaVelBindex)(void*,int);
void*(btGImpactMeshShapePart_getTrimeshPrimitiveManager)(void*);
float(btCollisionWorld_ContactResultCallback_addSingleResult)(void*,void*,const void*,int,int,const void*,int,int);
void(btMultiBodySolverConstraint_setDeltaVelAindex)(void*,int);
void(btMultiBodySolverConstraint_setContactNormal2)(void*,const void*);
void(btMultiBodySolverConstraint_setContactNormal1)(void*,const void*);
int(btQuantizedBvhNode_getEscapeIndexOrTriangleIndex)(void*);
void(btMultiBodySolverConstraint_setAppliedPushImpulse)(void*,float);
void(btMultiBodySolverConstraint_setAppliedImpulse)(void*,float);
void(btMultiBodySolverConstraint_setAngularComponentA)(void*,const void*);
void(btConvexCast_CastResult_setHitTransformA)(void*,const void*);
bool(btHingeConstraint_getUseFrameOffset)(void*);
float(btMultiBodySolverConstraint_getUpperLimit)(void*);
float(btMultiBodySolverConstraint_getUnusedPadding4)(void*);
void(btRotationalLimitMotor2_setMotorCFM)(void*,float);
void(btContactSolverInfoData_setMaxErrorReduction)(void*,float);
int(btCompoundShape_getUpdateRevision)(void*);
int(btMultiBodySolverConstraint_getSolverBodyIdA)(void*);
float(btMultiBodySolverConstraint_getRhsPenetration)(void*);
void*(btQuantizedBvh_new)();
void(btBoxBoxDetector_setBox2)(void*,const void*);
void(btMultiBodySolverConstraint_getRelpos2CrossNormal)(void*,void*);
void(btGeneric6DofConstraint_getAxis)(void*,int,void*);
void(btBroadphaseInterface_rayTest3)(void*,const void*,const void*,void*,const void*,const void*);
void(btDbvt_ICollide_delete)(void*);
void(btContactSolverInfoData_setGlobalCfm)(void*,float);
bool(btAxisSweep3_testAabbOverlap)(void*,void*,void*);
void*(btMultiBodySolverConstraint_getMultiBodyA)(void*);
void*(btSoftBodyHelpers_CreatePatchUV2)(void*,const void*,const void*,const void*,const void*,int,int,int,bool,float*);
int(btMultiBodySolverConstraint_getLinkB)(void*);
int(btMultiBodySolverConstraint_getLinkA)(void*);
int(btMultiBodySolverConstraint_getJacBindex)(void*);
void(btDbvtNode_setDataAsInt)(void*,int);
void*(btConvexPlaneCollisionAlgorithm_CreateFunc_new)();
float(btSliderConstraint_getRestitutionLimLin)(void*);
void(btDbvtBroadphase_optimize)(void*);
void(btMultiBodySolverConstraint_getContactNormal1)(void*,void*);
void*(btManifoldResult_getPersistentManifold)(void*);
float(btMultiBodySolverConstraint_getAppliedPushImpulse)(void*);
float(btMultiBody_getAngularDamping)(void*);
float(btMultiBodySolverConstraint_getAppliedImpulse)(void*);
void(btMultiBodySolverConstraint_getAngularComponentB)(void*,void*);
const void*(btDbvt_sStkNN_getB)(void*);
void(btSoftBody_setBUpdateRtCst)(void*,bool);
void(btMultiBodyPoint2Point_setPivotInB)(void*,const void*);
void(btMultiBodyPoint2Point_getPivotInB)(void*,void*);
void(btConeTwistConstraint_getMotorTarget)(void*,void*);
int(btConeShape_getConeUpIndex)(void*);
void*(btAlignedObjectArray_btSoftBody_Face_at)(void*,int);
int(btSoftBody_Impulse_getAsVelocity)(void*);
void(btMultiBodyLinkCollider_setLink)(void*,int);
void*(btMultiBodyLinkCollider_getMultiBody)(void*);
bool(btHingeConstraint_getEnableAngularMotor)(void*);
bool(btCollisionWorld_ContactResultCallback_needsCollision)(void*,void*);
void(btDbvtAabbMm_SignedExpand)(void*,const void*);
void*(btMultiBodyLinkCollider_new)(void*,int);
void*(btRaycastVehicle_addWheel)(void*,const void*,const void*,const void*,float,float,const void*,bool);
void(btMultibodyLink_setZeroRotParentToThis)(void*,const void*);
void(btMultiBody_setWorldToBaseRot)(void*,const void*);
void(btConstraintSolver_delete)(void*);
int(btGeneric6DofConstraint_get_limit_motor_info22)(void*,void*,const void*,const void*,const void*,const void*,const void*,const void*,void*,int,void*,int,int);
void(btMultibodyLink_setParent)(void*,int);
void(btSoftBody_updatePose)(void*);
void(btGhostObject_addOverlappingObjectInternal2)(void*,void*,void*);
void(btMultibodyLink_setLinkName)(void*,const char*);
void(btMultibodyLink_setJointType)(void*,int);
void(btMultibodyLink_setJointName)(void*,const char*);
void(btConvexShape_localGetSupportingVertexWithoutMargin)(void*,const void*,void*);
void(btSoftBody_Cluster_getCom)(void*,void*);
void(btGeneric6DofSpring2Constraint_enableMotor)(void*,int,bool);
void(btMultibodyLink_setJointFeedback)(void*,void*);
void(btAlignedObjectArray_btSoftBody_Face_push_back)(void*,void*);
float(btSoftBody_Config_getKDP)(void*);
bool(btConeTwistConstraint_isMotorEnabled)(void*);
void(btCollisionObjectWrapper_setShape)(void*,const void*);
void*(btDefaultMotionState_getUserPointer)(void*);
void(btConeTwistConstraint_calcAngleInfo)(void*);
short(btCollisionWorld_ContactResultCallback_getCollisionFilterMask)(void*);
int(btSoftBody_Config_getCiterations)(void*);
void(btDefaultCollisionConstructionInfo_delete)(void*);
bool(btDispatcher_needsResponse)(void*,const void*,const void*);
void(btDynamicsWorld_setInternalTickCallback2)(void*,void*,void*);
bool(btRotationalLimitMotor2_getEnableSpring)(void*);
void*(btRigidBody_btRigidBodyConstructionInfo_getMotionState)(void*);
void*(btDefaultCollisionConstructionInfo_getPersistentManifoldPool)(void*);
void*(btHingeAccumulatedAngleConstraint_new7)(void*,const void*);
bool(btConvexPenetrationDepthSolver_calcPenDepth)(void*,void*,const void*,const void*,const void*,const void*,void*,void*,void*,void*);
void(btAlignedObjectArray_btSoftBody_ClusterPtr_resizeNoInitialize)(void*,int);
void(btBox2dShape_getHalfExtentsWithMargin)(void*,void*);
void(btAngularLimit_set4)(void*,float,float,float,float,float);
void*(btAlignedObjectArray_btPersistentManifoldPtr_at)(void*,int);
void*(btDefaultCollisionConstructionInfo_getCollisionAlgorithmPool)(void*);
void*(btFace_getIndices)(void*);
void(btMultiBody_setPosUpdated)(void*,bool);
short(btBroadphaseProxy_getCollisionFilterGroup)(void*);
void(btDbvt_remove)(void*,void*);
int(btMultiBodySolverConstraint_getOverrideNumSolverIterations)(void*);
void*(btHinge2Constraint_new)(void*,void*,void*,void*,void*);
void(btRigidBody_getLocalInertia)(void*,void*);
void(btGImpactQuantizedBvh_setPrimitiveManager)(void*,void*);
int(btDbvtBroadphase_getDupdates)(void*);
const unsigned char*(btMaterialProperties_getMaterialBase)(void*);
void(btUniversalConstraint_getAxis1)(void*,void*);
int(btDbvtBroadphase_getNewpairs)(void*);
void(btDefaultCollisionConfiguration_setConvexConvexMultipointIterations2)(void*,int);
void(btDbvt_sStkNP_setMask)(void*,int);
void*(btCompoundShapeChild_getNode)(void*);
int(btConvexPlaneCollisionAlgorithm_CreateFunc_getNumPerturbationIterations)(void*);
bool(btDbvtBroadphase_getNeedcleanup)(void*);
void(btCompoundShapeChild_setChildShape)(void*,void*);
void(btSoftBody_setWorldInfo)(void*,void*);
float(btCollisionShape_getContactBreakingThreshold)(void*,float);
void*(btAlignedObjectArray_btSoftBody_Node_at)(void*,int);
void(btDbvt_benchmark)();
int(btMultiBody_getNumPosVars)(void*);
float(btMultiBody_getJointPos)(void*,int);
void(btMultiBody_getBaseTorque)(void*,void*);
float(btDbvt_sStkNPS_getValue)(void*);
void(btDynamicsWorld_addRigidBody2)(void*,void*,short,short);
void(btDbvt_sStkNP_setNode)(void*,const void*);
void(btDynamicsWorld_setInternalTickCallback)(void*,void*);
void(btMultiBody_fillContactJacobianMultiDof)(void*,int,const void*,const void*,float*,void*,void*,void*);
int(btDefaultCollisionConstructionInfo_getCustomCollisionAlgorithmMaxElementSize)(void*);
float*(btTypedConstraint_btConstraintInfo2_getCfm)(void*);
void*(btMultiBodyDynamicsWorld_getMultiBodyConstraint)(void*,int);
void(btContactConstraint_setContactManifold)(void*,void*);
float(btSliderConstraint_getSoftnessLimAng)(void*);
void*(btDbvt_sStkNN_new)();
bool(btCollisionShape_isPolyhedral)(void*);
void(btDbvt_sStkCLN_setNode)(void*,const void*);
void*(btCompoundCompoundCollisionAlgorithm_new)(const void*,const void*,const void*,bool);
void(btStridingMeshInterface_getLockedReadOnlyVertexIndexBase2)(void*,const unsigned char**,int*,int*,int*,const unsigned char**,int*,int*,int*,int);
void(btGImpactCollisionAlgorithm_gimpact_vs_compoundshape)(void*,const void*,const void*,const void*,const void*,bool);
void**(btDbvtNode_getChilds)(void*);
int(btDbvtBroadphase_getFupdates)(void*);
float(btRotationalLimitMotor_getBounce)(void*);
void*(btDefaultSoftBodySolver_new)();
void*(btCompoundCollisionAlgorithm_getChildAlgorithm)(void*,int);
void(btMinkowskiSumShape_getTransformA)(void*,void*);
float(btContactSolverInfoData_getTimeStep)(void*);
void(btConvexCast_CastResult_setDebugDrawer)(void*,void*);
void(btGeneric6DofSpring2Constraint_setEquilibriumPoint2)(void*,int,float);
float(btWheelInfo_getClippedInvContactDotSuspension)(void*);
void(btCollisionWorld_RayResultCallback_setCollisionFilterGroup)(void*,short);
void*(btBroadphasePair_getPProxy0)(void*);
void(btContactSolverInfoData_setFriction)(void*,float);
void(btBvhTriangleMeshShape_setOptimizedBvh2)(void*,void*,const void*);
void*(btCollisionAlgorithmConstructionInfo_getDispatcher1)(void*);
void(btRotationalLimitMotor2_setBounce)(void*,float);
void(btDbvt_setLeaves)(void*,int);
bool(btBroadphaseProxy_isSoftBody)(int);
void(btDynamicsWorld_getGravity)(void*,void*);
void(btManifoldPoint_getLocalPointB)(void*,void*);
float(btAngularLimit_getHalfRange)(void*);
int(btManifoldPoint_getLifeTime)(void*);
void(btBvhTriangleMeshShape_setTriangleInfoMap)(void*,void*);
float(btDbvtBroadphase_getPrediction)(void*);
float(btCollisionObject_getFriction)(void*);
void(btSliderConstraint_setFrames)(void*,const void*,const void*);
void(btConvexPolyhedron_setLocalCenter)(void*,const void*);
void(btRigidBody_proceedToTransform)(void*,const void*);
void(btSoftBody_initializeFaceTree)(void*);
bool(btMultiBody_isUsingGlobalVelocities)(void*);
int(btConvexHullShape_getNumPoints)(void*);
float(btManifoldPoint_getAppliedImpulseLateral1)(void*);
float(btContactSolverInfoData_getSplitImpulsePenetrationThreshold)(void*);
void(btConvexInternalShape_setSafeMargin3)(void*,const void*);
void(btConvexPolyhedron_getMC)(void*,void*);
void(btConeTwistConstraint_setMaxMotorImpulseNormalized)(void*,float);
const char*(btMultibodyLink_getLinkName)(void*);
void(btConvexPolyhedron_getLocalCenter)(void*,void*);
void(btCollisionWorld_contactPairTest)(void*,void*,void*,void*);
bool(btCollisionObject_hasAnisotropicFriction2)(void*,int);
void(btConvexPointCloudShape_setPoints2)(void*,void*,int,bool);
void(btAABB_appy_transform_trans_cache)(void*,const void*);
void(btCollisionObject_setIslandTag)(void*,int);
void(btCompoundShapeChild_setTransform)(void*,const void*);
void(btCollisionWorld_RayResultCallback_setCollisionObject)(void*,const void*);
void(btDbvt_update2)(void*,void*);
void*(btSortedOverlappingPairCache_getOverlapFilterCallback)(void*);
void(btAlignedObjectArray_btSoftBody_Anchor_push_back)(void*,void*);
float(btWheelInfo_getBrake)(void*);
void*(btBvhTriangleMeshShape_new)(void*,bool);
void*(btAlignedObjectArray_btBroadphasePair_at)(void*,int);
float(btRotationalLimitMotor_getCurrentLimitError)(void*);
void(btRigidBody_getLinearVelocity)(void*,void*);
void*(btGImpactQuantizedBvh_new)();
float*(btTypedConstraint_btConstraintInfo2_getJ1linearAxis)(void*);
void(btDefaultMotionState_setUserPointer)(void*,void*);
int(btConvexConvexAlgorithm_CreateFunc_getMinimumPointsPerturbationThreshold)(void*);
int*(btTranslationalLimitMotor2_getCurrentLimit)(void*);
void*(btConvexConvexAlgorithm_new)(void*,const void*,const void*,const void*,void*,void*,int,int);
void(btConvexConvexAlgorithm_CreateFunc_setPdSolver)(void*,void*);
float(btConvexPolyhedron_getRadius)(void*);
void(btIndexedMesh_setIndexType)(void*,int);
unsigned int(btDbvtBroadphase_getUpdates_call)(void*);
void*(btDispatcher_allocateCollisionAlgorithm)(void*,int);
void*(btSoftBody_Cluster_getLeaf)(void*);
void(btHingeConstraint_getInfo2Internal)(void*,void*,const void*,const void*,const void*,const void*);
void*(btCylinderShapeX_new2)(float,float,float);
void(btMultiBodyDynamicsWorld_integrateTransforms)(void*,float);
void(btMultibodyLink_getCachedRVector)(void*,void*);
void(btConvexCast_CastResult_setNormal)(void*,const void*);
void(btHingeConstraint_enableAngularMotor)(void*,bool,float,float);
int(btGImpactShapeInterface_getGImpactShapeType)(void*);
const void*(btManifoldResult_getBody0Internal)(void*);
void(btBroadphaseInterface_delete)(void*);
void*(btDbvt_sStkNN_new2)(const void*,const void*);
void(btMultiBody_calcAccelerationDeltasMultiDof)(void*,const float*,float*,void*,void*);
void*(btCompoundShape_getChildShape)(void*,int);
void(btCollisionShape_setMargin)(void*,float);
void(btConvexCast_CastResult_getHitPoint)(void*,void*);
float(btCollisionObject_getRestitution)(void*);
void*(btSoftBody_getFaces)(void*);
void(btVoronoiSimplexSolver_getCachedP1)(void*,void*);
float(btRigidBody_getLinearSleepingThreshold)(void*);
void(btConvex2dConvex2dAlgorithm_CreateFunc_setPdSolver)(void*,void*);
void(btGjkPairDetector_setFixContactNormalDirection)(void*,int);
int(btGImpactQuantizedBvh_getNodeCount)(void*);
void(btGhostObject_convexSweepTest2)(void*,const void*,const void*,const void*,void*,float);
float(btConvexInternalShape_getMarginNV)(void*);
void(btManifoldPoint_setPartId0)(void*,int);
void(btGImpactMeshShapePart_TrimeshPrimitiveManager_lock)(void*);
void(btGImpactMeshShapePart_TrimeshPrimitiveManager_getScale)(void*,void*);
void*(btMultiBodySolverConstraint_new)();
int(btGjkPairDetector_getDegenerateSimplex)(void*);
void(btContactSolverInfoData_setSplitImpulse)(void*,int);
void(btContactSolverInfoData_setSor)(void*,float);
float(btConeTwistConstraint_getLimit)(void*,int);
float(btRotationalLimitMotor2_getCurrentPosition)(void*);
void(btConvexPolyhedron_setME)(void*,const void*);
void(btConvexTriangleCallback_setTriangleCount)(void*,int);
void(btContactSolverInfoData_setSingleAxisRollingFrictionThreshold)(void*,float);
void*(btDbvtAabbMm_FromCR)(const void*,float);
void*(btGhostObject_upcast)(void*);
void(btGImpactMeshShapePart_TrimeshPrimitiveManager_setNumfaces)(void*,int);
void(btDbvt_optimizeTopDown)(void*);
void*(btFixedConstraint_new)(void*,void*,const void*,const void*);
void(btTriangleMeshShape_getLocalAabbMin)(void*,void*);
void*(btConvex2dConvex2dAlgorithm_CreateFunc_getSimplexSolver)(void*);
void*(btMultiBodySolverConstraint_getOrgConstraint)(void*);
void(btBroadphaseProxy_setCollisionFilterGroup)(void*,short);
void(btGImpactMeshShapePart_TrimeshPrimitiveManager_setIndexbase)(void*,const unsigned char*);
void(btContactSolverInfoData_setErp2)(void*,float);
void(btGeometryUtil_getVerticesFromPlaneEquations)(const void*,void*);
void*(btGhostObject_getOverlappingPairs)(void*);
float(btContactSolverInfoData_getWarmstartingFactor)(void*);
void(btDefaultCollisionConstructionInfo_setDefaultMaxPersistentManifoldPoolSize)(void*,int);
float(btContactSolverInfoData_getMaxGyroscopicForce)(void*);
float(btContactSolverInfoData_getMaxErrorReduction)(void*);
void(btDefaultMotionState_getGraphicsWorldTrans)(void*,void*);
int(btDefaultCollisionConstructionInfo_getDefaultMaxCollisionAlgorithmPoolSize)(void*);
float(btContactSolverInfoData_getDamping)(void*);
void*(btConvex2dShape_getChildShape)(void*);
void(btDbvt_extractLeaves)(const void*,void*);
void(btGImpactShapeInterface_setChildTransform)(void*,int,const void*);
void(btBroadphaseRayCallback_getRayDirectionInverse)(void*,void*);
void(btCollisionObjectWrapper_setCollisionObject)(void*,const void*);
void(btConvexInternalShape_setSafeMargin)(void*,float);
void*(btCylinderShapeZ_new)(const void*);
void(btDynamicsWorld_addRigidBody)(void*,void*);
void(btConeTwistConstraint_setLimit4)(void*,float,float,float,float,float);
void(btKinematicCharacterController_setGravity)(void*,float);
bool(btCharacterControllerInterface_onGround)(void*);
void(btSoftBodyHelpers_DrawFaceTree3)(void*,void*,int,int);
void(btCylinderShape_getHalfExtentsWithMargin)(void*,void*);
void*(btDefaultCollisionConfiguration_getSimplexSolver)(void*);
void(btGImpactShapeInterface_getChildTransform)(void*,int,void*);
void(btTranslationalLimitMotor_setRestitution)(void*,float);
void(btDiscreteCollisionDetectorInterface_delete)(void*);
bool(btDispatcherInfo_getUseContinuous)(void*);
bool(btMultiBody_internalNeedsJointFeedback)(void*);
void*(btWorldImporter_createConeShapeZ)(void*,float,float);
int(btChunk_getLength)(void*);
void(btTranslationalLimitMotor2_getMaxMotorForce)(void*,void*);
void(btCollisionObject_setBroadphaseHandle)(void*,void*);
void*(btGearConstraint_new2)(void*,void*,const void*,const void*,float);
float(btMultibodyLink_getJointFriction)(void*);
void(btDbvtBroadphase_setFupdates)(void*,int);
void*(btCollisionDispatcher_getNearCallback)(void*);
void(btDispatcher_delete)(void*);
void(btCollisionObject_delete)(void*);
void*(btDbvt_ICollide_new)();
void(btCompoundShape_removeChildShape)(void*,void*);
void*(btCollisionObject_new)();
void*(btCollisionWorld_ConvexResultCallbackWrapper_new)(void*,void*);
void*(btKinematicCharacterController_new2)(void*,void*,float,int);
float(btConeTwistConstraint_getFixThresh)(void*);
void*(btDispatcher_getInternalManifoldPool)(void*);
void*(btPoint2PointConstraint_new2)(void*,const void*);
void*(btConeTwistConstraint_new)(void*,void*,const void*,const void*);
int(btSoftBody_Link_getBbending)(void*);
void(btConeShape_setConeUpIndex)(void*,int);
void(btQuantizedBvh_quantizeWithClamp)(void*,unsigned short*,const void*,int);
float(btRigidBody_btRigidBodyConstructionInfo_getAngularSleepingThreshold)(void*);
void(btGImpactMeshShapePart_TrimeshPrimitiveManager_setNumverts)(void*,int);
void(btWheelInfoConstructionInfo_delete)(void*);
void(btCompoundShape_recalculateLocalAabb)(void*);
void(btSimulationIslandManager_initUnionFind)(void*,int);
void(btGeneric6DofSpringConstraint_setEquilibriumPoint)(void*);
float(btCollisionWorld_LocalConvexResult_getHitFraction)(void*);
void*(btAABB_new4)(const void*);
void*(btDiscreteDynamicsWorld_getSimulationIslandManager)(void*);
void(btManifoldPoint_setContactCFM)(void*,float);
float(btCollisionObject_getCcdSquareMotionThreshold)(void*);
void(btCompoundShapeChild_setNode)(void*,void*);
float(btConvexShape_getMarginNonVirtual)(void*);
void(btGImpactCollisionAlgorithm_setFace0)(void*,int);
void*(btBroadphaseAabbCallbackWrapper_new)(void*);
float(btCapsuleShape_getRadius)(void*);
void(btCollisionObject_setRollingFriction)(void*,float);
void(btGImpactQuantizedBvh_setNodeBound)(void*,int,const void*);
void(btCompoundShapeChild_getTransform)(void*,void*);
void(btBroadphaseInterface_resetPool)(void*,void*);
void*(btSoftBody_getClusterConnectivity)(void*);
void(btMultibodyLink_getZeroRotParentToThis)(void*,void*);
void(btTriangleMeshShape_recalcLocalAabb)(void*);
void(btGeneric6DofSpring2Constraint_setStiffness)(void*,int,float);
void*(btCompoundCompoundCollisionAlgorithm_CreateFunc_new)();
float(btTypedConstraint_btConstraintInfo2_getDamping)(void*);
void(btCollisionObject_setCcdMotionThreshold)(void*,float);
void*(btTriangleIndexVertexArray_new2)(int,int*,int,int,float*,int);
void(btTypedConstraint_setUserConstraintId)(void*,int);
float(btDiscreteCollisionDetectorInterface_ClosestPointInput_getMaximumDistanceSquared)(void*);
void(btDbvt_ICollide_Process2)(void*,const void*);
void*(btGImpactBvh_new2)(void*);
int(btQuantizedBvhTree_getLeftNode)(void*,int);
float(btSoftBody_Pose_getVolume)(void*);
float(btConstraintSetting_getTau)(void*);
void(btGImpactMeshShapePart_TrimeshPrimitiveManager_setVertexbase)(void*,const unsigned char*);
void(btSoftBodyHelpers_DrawClusterTree3)(void*,void*,int,int);
void(btSoftBody_SContact_getWeights)(void*,void*);
void*(btSoftBodyHelpers_CreateFromConvexHull)(void*,const float*,int);
void(btDbvt_IWriter_Prepare)(void*,const void*,int);
void(btAlignedObjectArray_btSoftBody_Link_resizeNoInitialize)(void*,int);
void(btCollisionShape_setLocalScaling)(void*,const void*);
int(btSoftBody_getLinkVertexNormalData)(void*,float*);
void(btSoftBody_addAeroForceToNode)(void*,const void*,int);
int(btGeneric6DofConstraint_get_limit_motor_info2)(void*,void*,const void*,const void*,const void*,const void*,const void*,const void*,void*,int,void*,int);
void*(btCollisionWorld_LocalRayResult_new)(const void*,void*,const void*,float);
void(btGImpactShapeInterface_getPrimitiveTriangle)(void*,int,void*);
void*(btMultiBodyPoint2Point_new2)(void*,int,void*,int,const void*,const void*);
bool(btAABB_overlapping_trans_cache)(void*,const void*,const void*,bool);
void*(btPolarDecomposition_new2)(float);
void(btCollisionWorld_LocalConvexResult_getHitNormalLocal)(void*,void*);
float(btGeneric6DofSpringConstraint_getStiffness)(void*,int);
short(btCollisionWorld_ConvexResultCallback_getCollisionFilterGroup)(void*);
void(btManifoldResult_refreshContactPoints)(void*);
void(btCollisionWorld_addCollisionObject2)(void*,void*,short);
void(btCollisionWorld_ClosestConvexResultCallback_setConvexFromWorld)(void*,const void*);
void(btConvexCast_delete)(void*);
float(btSoftBodySolver_getTimeScale)(void*);
void(btGeneric6DofSpring2Constraint_getAngularUpperLimitReversed)(void*,void*);
void*(btDbvt_sStkNPS_new2)(const void*,unsigned int,float);
void(btConvexPointCloudShape_setPoints)(void*,void*,int);
void(btCollisionWorld_contactTest)(void*,void*,void*);
void(btDbvtAabbMm_Center)(void*,void*);
int(btMultiBody_getNumDofs)(void*);
int(btDbvt_allocate)(void*,void*,const void*);
void(btCollisionWorld_ClosestConvexResultCallback_setHitPointWorld)(void*,const void*);
void(btCollisionWorld_ClosestConvexResultCallback_setHitNormalWorld)(void*,const void*);
void(btSoftBodySolver_solveConstraints)(void*,float);
void(btCollisionWorld_objectQuerySingle)(const void*,const void*,const void*,void*,const void*,const void*,void*,float);
void(btDbvtAabbMm_tMins)(void*,void*);
void(btGeneric6DofSpringConstraint_setDamping)(void*,int,float);
void(btCollisionWorld_ClosestConvexResultCallback_getHitPointWorld)(void*,void*);
void*(btCollisionConfiguration_getPersistentManifoldPool)(void*);
void(btOptimizedBvh_refitPartial)(void*,void*,const void*,const void*);
void(btSoftBody_addForce2)(void*,const void*,int);
void*(btHingeAccumulatedAngleConstraint_new8)(void*,const void*,bool);
void*(btWorldImporter_createTriangleInfoMap)(void*);
void(btAxisSweep3_setOverlappingPairUserCallback)(void*,void*);
float(btConeShape_getHeight)(void*);
float(btRaycastVehicle_btVehicleTuning_getSuspensionStiffness)(void*);
void(btBroadphasePair_delete)(void*);
void(btSoftBody_defaultCollisionHandler)(void*,const void*);
float(btRotationalLimitMotor2_getMotorERP)(void*);
void(btGeneric6DofSpring2Constraint_getLinearUpperLimit)(void*,void*);
float(btManifoldPoint_getDistance1)(void*);
void(btGeneric6DofSpring2Constraint_setEquilibriumPoint3)(void*,int);
float(btCollisionObject_getDeactivationTime)(void*);
void(btCollisionObject_setRestitution)(void*,float);
void(btRotationalLimitMotor2_setSpringDampingLimited)(void*,bool);
void(btBoxBoxDetector_setBox1)(void*,const void*);
void(btCollisionWorld_LocalRayResult_setHitFraction)(void*,float);
void*(btCollisionObject_internalGetExtensionPointer)(void*);
int(btMultiBodyConstraint_getIslandIdA)(void*);
void(btPointCollector_setPointInWorld)(void*,const void*);
float(btGeneric6DofSpring2Constraint_getAngle)(void*,int);
void(btQuantizedBvhTree_getNodeBound)(void*,int,void*);
void(btDefaultCollisionConfiguration_setPlaneConvexMultipointIterations)(void*);
void(btCompoundShape_addChildShape)(void*,const void*,void*);
void(btSoftBody_evaluateCom)(void*,void*);
void(btConeTwistConstraint_getInfo2NonVirtual)(void*,void*,const void*,const void*,const void*,const void*);
void(btBoxShape_getPlaneEquation)(void*,void*,int);
void(btGImpactBvh_update)(void*);
void(btAlignedObjectArray_btSoftBody_Link_push_back)(void*,void*);
void(btCollisionObject_setInterpolationWorldTransform)(void*,const void*);
int(btConvex2dConvex2dAlgorithm_CreateFunc_getNumPerturbationIterations)(void*);
void(btConvexShape_getAabbSlow)(void*,const void*,void*,void*);
void*(btMultibodyLink_getAbsFrameTotVelocity)(void*);
void(btCollisionWorld_LocalConvexResult_setLocalShapeInfo)(void*,void*);
void(btMultiBody_setMaxAppliedImpulse)(void*,float);
void(btTranslationalLimitMotor2_getBounce)(void*,void*);
void*(btBvhTriangleMeshShape_new3)(void*,bool,const void*,const void*);
void(btMultiBodyConstraint_delete)(void*);
int(btGImpactMeshShapePart_TrimeshPrimitiveManager_getLock_count)(void*);
void(btSliderConstraint_setRestitutionLimLin)(void*,float);
void(btCollisionWorld_ClosestRayResultCallback_getHitPointWorld)(void*,void*);
void(btKinematicCharacterController_setMaxSlope)(void*,float);
void(btPairSet_push_pair_inv)(void*,int,int);
int(btCollisionObjectWrapper_getIndex)(void*);
void(btConeTwistConstraint_setMaxMotorImpulse)(void*,float);
int(btDbvtBroadphase_getCid)(void*);
void(btHinge2Constraint_setLowerLimit)(void*,float);
void(btBroadphaseInterface_printStats)(void*);
void*(btBoxBoxCollisionAlgorithm_new)(const void*);
void(btCollisionWorld_addCollisionObject)(void*,void*);
int(btAlignedObjectArray_btBroadphasePair_size)(void*);
void(btRigidBody_applyGravity)(void*);
int(btContactSolverInfoData_getSolverMode)(void*);
void*(btTriangleCallbackWrapper_new)(void*);
void(btConvexInternalShape_setSafeMargin2)(void*,float,float);
float(btConeTwistConstraint_getLimitSoftness)(void*);
void(btBroadphaseProxy_getAabbMax)(void*,void*);
void(btGeneric6DofSpring2Constraint_setLimit)(void*,int,float,float);
float(btManifoldPoint_getFrictionCFM)(void*);
void(btMultibodyLink_setCollider)(void*,void*);
int(btAlignedObjectArray_btSoftBody_Face_size)(void*);
void(btManifoldPoint_setDistance)(void*,float);
bool(btCollisionWorld_ConvexResultCallback_hasHit)(void*);
int(btBroadphaseProxy_getUniqueId)(void*);
const char*(btMultiBody_serialize)(void*,void*,void*);
void*(btMultiBodyDynamicsWorld_new)(void*,void*,void*,void*);
void(btCollisionWorld_AllHitsRayResultCallback_setRayToWorld)(void*,const void*);
float(btWheelInfo_getWheelsDampingRelaxation)(void*);
void(btSparseSdf3_Initialize)(void*,int,int);
void*(btAlignedObjectArray_btSoftBody_MaterialPtr_at)(void*,int);
float(btSoftBody_Tetra_getC2)(void*);
int(btDispatcherInfo_getDispatchFunc)(void*);
void(btAlignedObjectArray_btSoftBody_Face_resizeNoInitialize)(void*,int);
void*(btHingeConstraint_new5)(void*,void*,const void*,const void*);
int(btConvexConvexAlgorithm_CreateFunc_getNumPerturbationIterations)(void*);
void(btDiscreteCollisionDetectorInterface_ClosestPointInput_delete)(void*);
float(btRotationalLimitMotor2_getStopCFM)(void*);
void*(btCollisionWorld_AllHitsRayResultCallback_new)(const void*,const void*);
void(btDbvtBroadphase_performDeferredRemoval)(void*,void*);
void(btCollisionObject_getInterpolationAngularVelocity)(void*,void*);
void(btGeneric6DofSpringConstraint_setEquilibriumPoint2)(void*,int);
void(btConvexPlaneCollisionAlgorithm_collideSingleContact)(void*,const void*,const void*,const void*,const void*,void*);
void*(btAABB_new3)(const void*,const void*,const void*,float);
void(btTranslationalLimitMotor2_getCurrentLimitErrorHi)(void*,void*);
float(btCollisionObject_getHitFraction)(void*);
void(btGImpactShapeInterface_updateBound)(void*);
void*(btCollisionWorld_AllHitsRayResultCallback_getHitPointWorld)(void*);
void*(btCompoundCompoundCollisionAlgorithm_SwappedCreateFunc_new)();
unsigned short*(btQuantizedBvhNode_getQuantizedAabbMax)(void*);
void*(btCollisionShape_getUserPointer)(void*);
void*(btGjkEpaPenetrationDepthSolver_new)();
int(btCollisionObject_getActivationState)(void*);
void*(btSoftRigidCollisionAlgorithm_CreateFunc_new)();
float(btGjkPairDetector_getCachedSeparatingDistance)(void*);
void(btCollisionAlgorithmCreateFunc_setSwapped)(void*,bool);
void(btCollisionObjectWrapper_setPartId)(void*,int);
void(btTranslationalLimitMotor2_getStopERP)(void*,void*);
bool(btCollisionObject_mergesSimulationIslands)(void*);
void(btPersistentManifold_removeContactPoint)(void*,int);
void*(btConvexTriangleCallback_getManifoldPtr)(void*);
bool(btCollisionAlgorithmCreateFunc_getSwapped)(void*);
int(btConstraintSolver_getSolverType)(void*);
void(btGeneric6DofConstraint_getInfo2NonVirtual)(void*,void*,const void*,const void*,const void*,const void*,const void*,const void*);
bool(btCollisionWorld_RayResultCallback_needsCollision)(void*,void*);
bool(btGImpactBvh_isTrimesh)(void*);
int(btGImpactMeshShapePart_TrimeshPrimitiveManager_getPart)(void*);
void(btHingeConstraint_setMotorTargetVelocity)(void*,float);
void(btCollisionWorld_ClosestRayResultCallback_setRayFromWorld)(void*,const void*);
void(btDbvt_ICollide_Process)(void*,const void*,const void*);
void(btGeneric6DofConstraint_getAngularUpperLimit)(void*,void*);
void(btConvexPenetrationDepthSolver_delete)(void*);
void*(btSoftBodyWorldInfo_getBroadphase)(void*);
void(btConvexConvexAlgorithm_CreateFunc_setSimplexSolver)(void*,void*);
void(btTypedConstraint_solveConstraintObsolete)(void*,void*,void*,float);
void*(btTypedConstraint_getFixedBody)();
short(btBroadphaseProxy_getCollisionFilterMask)(void*);
void(btCollisionWorld_ClosestRayResultCallback_setHitPointWorld)(void*,const void*);
float(btKinematicCharacterController_getMaxSlope)(void*);
void*(btSoftBodyRigidBodyCollisionConfiguration_new)();
float(btRotationalLimitMotor_getMaxLimitForce)(void*);
void(btGImpactMeshShapePart_TrimeshPrimitiveManager_setStride)(void*,int);
void(btAABB_find_intersection)(void*,const void*,void*);
void(btMultiBodyConstraint_allocateJacobiansMultiDof)(void*);
bool(btDiscreteDynamicsWorld_getApplySpeculativeContactRestitution)(void*);
void(btTranslationalLimitMotor_setLowerLimit)(void*,const void*);
bool(btAABB_overlapping_trans_conservative)(void*,const void*,void*);
void(btCollisionWorld_ClosestRayResultCallback_setHitNormalWorld)(void*,const void*);
void(btDbvtBroadphase_setDupdates)(void*,int);
void*(btCompoundShape_new)();
float(btTranslationalLimitMotor_getRestitution)(void*);
void(btConeTwistConstraint_getBFrame)(void*,void*);
void(btSoftBody_RayFromToCaster_getRayNormalizedDirection)(void*,void*);
int(btContactSolverInfoData_getSplitImpulse)(void*);
const void*(btCollisionObjectWrapper_getCollisionObject)(void*);
float(btGearConstraint_getRatio)(void*);
void(btCollisionObject_getInterpolationLinearVelocity)(void*,void*);
void(btBroadphaseProxy_setCollisionFilterMask)(void*,short);
float(btCollisionObject_getContactProcessingThreshold)(void*);
void(btRotationalLimitMotor_setMaxMotorForce)(void*,float);
void(btGearConstraint_getAxisB)(void*,void*);
void(btRotationalLimitMotor_setCurrentLimit)(void*,int);
int(btCollisionObject_getIslandTag)(void*);
void(btRotationalLimitMotor_setEnableMotor)(void*,bool);
int(btGeneric6DofConstraint_getFlags)(void*);
int(btGImpactCollisionAlgorithm_getPart0)(void*);
short(btCollisionWorld_ConvexResultCallback_getCollisionFilterMask)(void*);
bool(btGeneric6DofConstraint_getUseSolveConstraintObsolete)(void*);
void(btBroadphaseInterface_calculateOverlappingPairs)(void*,void*);
void(btVoronoiSimplexSolver_getCachedP2)(void*,void*);
void(btRigidBody_setNewBroadphaseProxy)(void*,void*);
void*(btGearConstraint_new)(void*,void*,const void*,const void*);
void(btTypedConstraint_enableFeedback)(void*,bool);
void*(btMultiSphereShape_new)(const float*,const float*,int);
void(btBroadphaseInterface_setAabb)(void*,void*,const void*,const void*,void*);
void*(btAABB_new5)(const void*,float);
float(btRotationalLimitMotor_getLoLimit)(void*);
void(btConeTwistConstraint_updateRHS)(void*,float);
void(btDiscreteCollisionDetectorInterface_ClosestPointInput_getTransformB)(void*,void*);
void(btGImpactQuantizedBvh_buildSet)(void*);
float(btRotationalLimitMotor_getTargetVelocity)(void*);
void(btCompoundShape_createAabbTreeFromChildren)(void*);
bool(btOverlapFilterCallback_needBroadphaseCollision)(void*,void*,void*);
float(btConstraintSetting_getDamping)(void*);
void(btTranslationalLimitMotor2_setMotorCFM)(void*,const void*);
void(btFace_delete)(void*);
void(btMultiBody_getBaseForce)(void*,void*);
int(btManifoldPoint_getIndex1)(void*);
bool(btBroadphaseAabbCallback_process)(void*,const void*);
void*(btKinematicCharacterController_getGhostObject)(void*);
void(btDbvtAabbMm_Extents)(void*,void*);
float(btRotationalLimitMotor2_getCurrentLimitError)(void*);
void*(btDbvtAabbMm_FromPoints)(const void**,int);
float(btHingeConstraint_getLimitSign)(void*);
void(btRaycastVehicle_getWheelTransformWS)(void*,int,void*);
void(btUsageBitfield_reset)(void*);
int(btMultiBodyLinkCollider_getLink)(void*);
void(btAlignedObjectArray_btSoftBody_Link_set)(void*,void*,int);
void(btConvexCast_CastResult_getHitTransformA)(void*,void*);
void*(btAlignedObjectArray_btSoftBody_ClusterPtr_at)(void*,int);
void*(btCollisionWorld_AllHitsRayResultCallback_getHitNormalWorld)(void*);
int(btAlignedObjectArray_btSoftBody_Link_size)(void*);
void(btDynamicsWorld_addConstraint2)(void*,void*,bool);
void(btTranslationalLimitMotor2_getMotorCFM)(void*,void*);
void(btConeTwistConstraint_GetPointForAngle)(void*,float,float,void*);
void(btConvexShape_localGetSupportVertexNonVirtual)(void*,const void*,void*);
void(btGImpactMeshShapePart_TrimeshPrimitiveManager_unlock)(void*);
void(btConvexPolyhedron_setRadius)(void*,float);
void*(btWheelInfo_getClientInfo)(void*);
void(btCollisionWorld_ClosestConvexResultCallback_setConvexToWorld)(void*,const void*);
void(btConvexPolyhedron_delete)(void*);
float(btHinge2Constraint_getAngle1)(void*);
float(btCompoundShapeChild_getChildMargin)(void*);
bool(btSortedOverlappingPairCache_needsBroadphaseCollision)(void*,void*,void*);
void(btCollisionWorld_ClosestRayResultCallback_getHitNormalWorld)(void*,void*);
void*(btCylinderShapeZ_new2)(float,float,float);
void(btCollisionWorld_ClosestRayResultCallback_getRayFromWorld)(void*,void*);
void(btCollisionWorld_ClosestRayResultCallback_getRayToWorld)(void*,void*);
void(btGImpactShapeInterface_getBulletTetrahedron)(void*,int,void*);
void(btSoftBody_indicesToPointers2)(void*,const int*);
void(btCollisionWorld_ContactResultCallback_delete)(void*);
void(btDbvtBroadphase_setDeferedcollide)(void*,bool);
void(btDbvt_ICollide_Process3)(void*,const void*,float);
float(btCollisionWorld_ConvexResultCallback_getClosestHitFraction)(void*);
void(btAlignedObjectArray_btPersistentManifoldPtr_resizeNoInitialize)(void*,int);
void(btCollisionWorld_addCollisionObject3)(void*,void*,short,short);
void*(btDbvt_new)();
void(btDispatcherInfo_setDispatchFunc)(void*,int);
void(btStorageResult_setDistance)(void*,float);
void(btGImpactMeshShapePart_TrimeshPrimitiveManager_setType)(void*,int);
int(btGImpactMeshShapePart_TrimeshPrimitiveManager_getType)(void*);
void(btBoxShape_getHalfExtentsWithMargin)(void*,void*);
void*(btConvexPlaneCollisionAlgorithm_new)(void*,const void*,const void*,const void*,bool,int,int);
void(btDiscreteDynamicsWorld_setSynchronizeAllMotionStates)(void*,bool);
void(btCollisionWorld_ClosestRayResultCallback_setRayToWorld)(void*,const void*);
int(btCollisionWorld_LocalShapeInfo_getTriangleIndex)(void*);
void(btSoftBody_Cluster_getAv)(void*,void*);
bool(btConeTwistConstraint_getAngularOnly)(void*);
void(btJointFeedback_getAppliedForceBodyB)(void*,void*);
void(btTranslationalLimitMotor2_setCurrentLinearDiff)(void*,const void*);
void(btCollisionAlgorithm_delete)(void*);
void(btCollisionWorld_LocalRayResult_setHitNormalLocal)(void*,const void*);
void(btCollisionWorld_LocalShapeInfo_setShapePart)(void*,int);
void*(btDbvtAabbMm_FromMM)(const void*,const void*);
void*(btWorldImporter_createPoint2PointConstraint)(void*,void*,const void*);
short(btCollisionWorld_RayResultCallback_getCollisionFilterMask)(void*);
int(btDispatcher_getNumManifolds)(void*);
void(btDiscreteCollisionDetectorInterface_Result_addContactPoint)(void*,const void*,const void*,float);
void(btConeTwistConstraint_setAngularOnly)(void*,bool);
void(btDiscreteCollisionDetectorInterface_Result_setShapeIdentifiersA)(void*,int,int);
void(btVehicleRaycaster_btVehicleRaycasterResult_getHitNormalInWorld)(void*,void*);
void*(btCollisionAlgorithmConstructionInfo_new2)(void*,int);
void*(btHingeAccumulatedAngleConstraint_new4)(void*,const void*,const void*,bool);
float(btStorageResult_getDistance)(void*);
int(btBvhTree_getLeftNode)(void*,int);
bool(btGeneric6DofConstraint_getUseLinearReferenceFrameA)(void*);
void(btDbvtAabbMm_Expand)(void*,const void*);
void(btMultiBody_getAngularMomentum)(void*,void*);
void*(btDiscreteDynamicsWorld_new)(void*,void*,void*,void*);
void(btConvexTriangleMeshShape_calculatePrincipalAxisTransform)(void*,void*,void*,float*);
float(btDispatcherInfo_getConvexConservativeDistanceThreshold)(void*);
void(btCompoundShapeChild_setChildMargin)(void*,float);
bool(btGeneric6DofConstraint_isLimited)(void*,int);
int(btDispatcherInfo_getStepCount)(void*);
float(btDispatcherInfo_getTimeStep)(void*);
void(btDispatcherInfo_setAllowedCcdPenetration)(void*,float);
void(btDispatcherInfo_setConvexConservativeDistanceThreshold)(void*,float);
void(btDispatcherInfo_setDebugDraw)(void*,void*);
void(btRotationalLimitMotor2_setStopERP)(void*,float);
void(btMultiBody_setBaseName)(void*,const char*);
void(btGeneric6DofSpring2Constraint_getFrameOffsetB)(void*,void*);
int(btGImpactBvh_getRightNode)(void*,int);
void(btDispatcherInfo_setUseConvexConservativeDistanceUtil)(void*,bool);
void(btTriangleMesh_setWeldingThreshold)(void*,float);
void(btDispatcher_clearManifold)(void*,void*);
void(btConvexConvexAlgorithm_CreateFunc_setNumPerturbationIterations)(void*,int);
void*(btMLCPSolver_new)(void*);
void*(btSoftBody_AJoint_IControlWrapper_getWrapperData)(void*);
void(btBvhTriangleMeshShape_partialRefitTree)(void*,const void*,const void*);
bool(btDbvt_update6)(void*,void*,void*,const void*,float);
void*(btDbvt_getRoot)(void*);
void(btDbvtBroadphase_setReleasepaircache)(void*,bool);
void(btSoftBody_Impulse_getVelocity)(void*,void*);
void*(btDynamicsWorld_getConstraint)(void*,int);
void*(btDynamicsWorld_getConstraintSolver)(void*);
void*(btBulletXmlWorldImporter_new)(void*);
const void*(btGImpactShapeInterface_getBoxSet)(void*);
void(btConeShape_setRadius)(void*,float);
void(btDynamicsWorld_setGravity)(void*,const void*);
int(btDynamicsWorld_stepSimulation)(void*,float);
void(btDbvt_optimizeTopDown2)(void*,int);
void(btSliderConstraint_getFrameOffsetB)(void*,void*);
void(btCollisionWorld_performDiscreteCollisionDetection)(void*);
void(btRotationalLimitMotor2_setCurrentLimitError)(void*,float);
void(btHingeConstraint_getBFrame)(void*,void*);
void(btDbvtAabbMm_Mins)(void*,void*);
void(btCollisionObject_setCompanionId)(void*,int);
void(btMultiBodyConstraint_createConstraintRows)(void*,void*,void*,const void*);
void(btConvexShape_localGetSupportingVertex)(void*,const void*,void*);
void(btGhostObject_removeOverlappingObjectInternal2)(void*,void*,void*,void*);
void(btGImpactMeshShapePart_TrimeshPrimitiveManager_get_bullet_triangle)(void*,int,void*);
void(btGearConstraint_setAxisB)(void*,void*);
void(btRaycastVehicle_applyEngineForce)(void*,float,int);
void*(btRotationalLimitMotor_new2)(const void*);
float(btSphereShape_getRadius)(void*);
void(btRotationalLimitMotor_setTargetVelocity)(void*,float);
int(btRotationalLimitMotor_getCurrentLimit)(void*);
const void*(btMinkowskiSumShape_getShapeB)(void*);
int(btBroadphaseProxy_getUid)(void*);
bool(btRotationalLimitMotor_getEnableMotor)(void*);
void(btCollisionWorld_LocalRayResult_setLocalShapeInfo)(void*,void*);
float(btRotationalLimitMotor2_getMaxMotorForce)(void*);
bool(btRotationalLimitMotor_needApplyTorques)(void*);
float(btRotationalLimitMotor_solveAngularLimits)(void*,float,void*,float,void*,void*);
void(btCollisionObject_setUserPointer)(void*,void*);
float(btRotationalLimitMotor_getMaxMotorForce)(void*);
void(btRotationalLimitMotor_setLoLimit)(void*,float);
void(btGImpactCompoundShape_CompoundPrimitiveManager_setCompoundShape)(void*,void*);
void(btMultibodyLink_setInertiaLocal)(void*,const void*);
float(btCollisionWorld_RayResultCallback_addSingleResult)(void*,void*,bool);
float(btContactSolverInfoData_getErp)(void*);
void(btMinkowskiSumShape_GetTransformB)(void*,void*);
void*(btHingeAccumulatedAngleConstraint_new5)(void*,void*,const void*,const void*);
void*(btGImpactQuantizedBvh_new2)(void*);
void(btRotationalLimitMotor_setLimitSoftness)(void*,float);
float(btConvexCast_CastResult_getFraction)(void*);
void(btBroadphaseProxy_setMultiSapParentProxy)(void*,void*);
void(btRotationalLimitMotor_setNormalCFM)(void*,float);
void(btRotationalLimitMotor_setStopCFM)(void*,float);
void(btStorageResult_getClosestPointInB)(void*,void*);
int*(btTranslationalLimitMotor_getCurrentLimit)(void*);
int(btMultiBodyDynamicsWorld_getNumMultibodies)(void*);
void*(btConvex2dConvex2dAlgorithm_CreateFunc_new)(void*,void*);
float(btWheelInfoConstructionInfo_getWheelsDampingCompression)(void*);
float(btTranslationalLimitMotor_getLimitSoftness)(void*);
void*(btGImpactCompoundShape_CompoundPrimitiveManager_new2)(void*);
void(btTranslationalLimitMotor_getTargetVelocity)(void*,void*);
void*(btMultibodyLink_getCollider)(void*);
void(btTranslationalLimitMotor_setAccumulatedImpulse)(void*,const void*);
void(btTriangleBuffer_clearBuffer)(void*);
void(btTranslationalLimitMotor_setDamping)(void*,float);
float(btMultiBody_getBaseMass)(void*);
void*(btMinkowskiSumShape_new)(const void*,const void*);
bool(btCollisionWorld_getForceUpdateAllAabbs)(void*);
int(btMultiBodySolverConstraint_getDeltaVelAindex)(void*);
void(btTranslationalLimitMotor_setMaxMotorForce)(void*,const void*);
void(btPersistentManifold_refreshContactPoints)(void*,const void*,const void*);
float(btDbvtAabbMm_ProjectMinimum)(void*,const void*,unsigned int);
void*(btHingeConstraint_new4)(void*,const void*,const void*,bool);
void(btTranslationalLimitMotor_setTargetVelocity)(void*,const void*);
bool(btMultiBody_isUsingRK4Integration)(void*);
void(btTranslationalLimitMotor_setUpperLimit)(void*,const void*);
void(btCollisionWorld_LocalConvexResult_getHitPointLocal)(void*,void*);
float(btTypedConstraint_internalGetAppliedImpulse)(void*);
void*(btSoftBodyRigidBodyCollisionConfiguration_new2)(const void*);
void(btGeneric6DofSpring2Constraint_setBounce)(void*,int,float);
void(btGeneric6DofSpringConstraint_setEquilibriumPoint3)(void*,int,float);
bool(btCollisionWorld_RayResultCallbackWrapper_needsCollision)(void*,void*);
void*(btHingeConstraint_new7)(void*,const void*);
void*(btMultiBodyLinkCollider_upcast)(void*);
void*(btGImpactMeshShapePart_TrimeshPrimitiveManager_new2)(const void*);
void*(btPairCachingGhostObject_getOverlappingPairCache)(void*);
void(btTriangleMesh_addTriangle)(void*,const void*,const void*,const void*);
bool(btSubSimplexClosestResult_getDegenerate)(void*);
bool(btGeneric6DofConstraint_getUseFrameOffset)(void*);
void(btCollisionShape_setUserIndex)(void*,int);
void(btDynamicsWorld_removeRigidBody)(void*,void*);
void*(btWorldImporter_createConeTwistConstraint)(void*,void*,void*,const void*,const void*);
void(btGeneric6DofConstraint_setLinearLowerLimit)(void*,const void*);
void(btGeneric6DofConstraint_setLinearUpperLimit)(void*,const void*);
void(btGImpactBvh_find_collision)(void*,const void*,void*,const void*,void*);
void*(btDbvt_sStkCLN_new)(const void*,void*);
float(btUniversalConstraint_getAngle2)(void*);
int(btDynamicsWorld_stepSimulation3)(void*,float,int,float);
bool(btVoronoiSimplexSolver_closest)(void*,void*);
float(btRotationalLimitMotor2_getLoLimit)(void*);
void*(btDbvt_getRayTestStack)(void*);
int(btPersistentManifold_addManifoldPoint)(void*,const void*);
void*(btCompoundShapeChild_getChildShape)(void*);
void(btCollisionDispatcher_setCollisionConfiguration)(void*,void*);
bool(btRotationalLimitMotor2_isLimited)(void*);
void(btRotationalLimitMotor2_setCurrentLimit)(void*,int);
void(btRotationalLimitMotor2_setCurrentLimitErrorHi)(void*,float);
void(btRotationalLimitMotor2_setCurrentPosition)(void*,float);
const unsigned char*(btGImpactMeshShapePart_TrimeshPrimitiveManager_getVertexbase)(void*);
int(btMultibodyLink_getDofCount)(void*);
void(btRotationalLimitMotor2_setEquilibriumPoint)(void*,float);
void(btRotationalLimitMotor2_setLoLimit)(void*,float);
int(btMultiBodySolverConstraint_getSolverBodyIdB)(void*);
void(btRotationalLimitMotor2_setServoMotor)(void*,bool);
void*(btCapsuleShapeX_new)(float,float);
short(btCollisionWorld_ContactResultCallback_getCollisionFilterGroup)(void*);
bool(btCollisionWorld_ContactResultCallbackWrapper_needsCollision)(void*,void*);
void(btGImpactQuantizedBvh_getNodeBound)(void*,int,void*);
void(btBroadphaseRayCallback_setRayDirectionInverse)(void*,const void*);
int(btDbvtBroadphase_getGid)(void*);
void(btMultiBody_setLinearDamping)(void*,float);
void(btTranslationalLimitMotor2_getCurrentLimitError)(void*,void*);
float(btCollisionAlgorithm_calculateTimeOfImpact)(void*,void*,void*,const void*,void*);
float(btGeneric6DofSpringConstraint_getEquilibriumPoint)(void*,int);
void(btSoftBody_clusterVelocity)(const void*,const void*,void*);
void(btMultiBody_setAngularDamping)(void*,float);
bool(btGImpactQuantizedBvh_isTrimesh)(void*);
int(btMultibodyLink_getFlags)(void*);
void(btSoftBody_Body_angularVelocity)(void*,const void*,void*);
void(btHinge2Constraint_setUpperLimit)(void*,float);
void*(btWorldImporter_createConeShapeX)(void*,float,float);
int(btShapeHull_numTriangles)(void*);
bool*(btTranslationalLimitMotor2_getSpringStiffnessLimited)(void*);
int(btVoronoiSimplexSolver_getNumVertices)(void*);
void(btGImpactMeshShapePart_TrimeshPrimitiveManager_setLock_count)(void*,int);
void(btDbvtBroadphase_setNewpairs)(void*,int);
void(btPersistentManifold_setBodies)(void*,const void*,const void*);
float(btKinematicCharacterController_getGravity)(void*);
void(btTranslationalLimitMotor2_setLowerLimit)(void*,const void*);
void(btTranslationalLimitMotor2_setMaxMotorForce)(void*,const void*);
void(btTranslationalLimitMotor2_setMotorERP)(void*,const void*);
void(btTranslationalLimitMotor2_setSpringDamping)(void*,const void*);
void(btTranslationalLimitMotor2_setSpringStiffness)(void*,const void*);
void(btTranslationalLimitMotor2_setStopCFM)(void*,const void*);
void(btTranslationalLimitMotor2_setTargetVelocity)(void*,const void*);
void(btTranslationalLimitMotor2_setUpperLimit)(void*,const void*);
void*(btSerializerWrapper_new)(void*,void*,void*,void*,void*,void*,void*,void*,void*,void*,void*,void*,void*,void*,void*);
void(btMultiBody_forwardKinematics)(void*,void*,void*);
void(btCollisionWorld_debugDrawWorld)(void*);
bool(btCollisionObject_hasAnisotropicFriction)(void*);
void(btGeneric6DofSpring2Constraint_getAngularUpperLimit)(void*,void*);
bool(btBulletXmlWorldImporter_loadFile)(void*,const char*);
void(btMultiBody_getBaseInertia)(void*,void*);
void(btGeneric6DofSpring2Constraint_getCalculatedTransformB)(void*,void*);
void(btDbvt_sStkNPS_setNode)(void*,const void*);
void(btManifoldPoint_setDistance1)(void*,float);
void(btMultiBodyConstraint_setMaxAppliedImpulse)(void*,float);
void*(btSparseSdf_new)();
void(btGeneric6DofSpring2Constraint_setEquilibriumPoint)(void*);
void(btGeneric6DofSpring2Constraint_setAngularUpperLimitReversed)(void*,const void*);
void(btCompoundShape_calculatePrincipalAxisTransform)(void*,float*,void*,void*);
float(btCollisionShape_getAngularMotionDisc)(void*);
void(btSoftBody_addAeroForceToFace)(void*,const void*,int);
void(btTranslationalLimitMotor_getStopCFM)(void*,void*);
const void*(btBvhTree_get_node_pointer2)(void*,int);
int(btCylinderShape_getUpAxis)(void*);
void(btRotationalLimitMotor2_setSpringStiffness)(void*,float);
void(btConeTwistConstraint_setDamping)(void*,float);
float(btConeTwistConstraint_getTwistSpan)(void*);
void(btDefaultCollisionConstructionInfo_setPersistentManifoldPool)(void*,void*);
void*(btTriangle_new)();
void(btGImpactQuantizedBvh_find_collision)(const void*,const void*,const void*,const void*,void*);
float(btCollisionObject_getCcdMotionThreshold)(void*);
void*(btStaticPlaneShape_new)(const void*,float);
void*(btGeneric6DofSpring2Constraint_new3)(void*,const void*);
void*(btSoftBody_Cluster_getFramerefs)(void*);
void(btConstraintSetting_delete)(void*);
void(btCollisionWorld_ContactResultCallback_setCollisionFilterMask)(void*,short);
void*(btGImpactCompoundShape_CompoundPrimitiveManager_new)(const void*);
void(btGjkPairDetector_setCatchDegeneracies)(void*,int);
void(btSoftBody_Node_setX)(void*,const void*);
void(btSoftBody_appendNode)(void*,const void*,float);
void(btCollisionAlgorithm_getAllContactManifolds)(void*,void*);
void(btTriangleInfoMap_setEqualVertexThreshold)(void*,float);
void(btCollisionDispatcher_registerCollisionCreateFunc)(void*,int,int,void*);
void(btCollisionWorld_LocalRayResult_setCollisionObject)(void*,const void*);
int(btSoftBody_Note_getRank)(void*);
void*(btTypedConstraint_getRigidBodyA)(void*);
float(btMultiBody_getMaxCoordinateVelocity)(void*);
void(btMultiBody_setupSpherical2)(void*,int,float,const void*,int,const void*,const void*,const void*,bool);
void(btChunk_setLength)(void*,int);
void(btTranslationalLimitMotor_setCurrentLinearDiff)(void*,const void*);
void(btRigidBody_computeGyroscopicForceExplicit)(void*,float,void*);
void(btGeneric6DofSpringConstraint_setStiffness)(void*,int,float);
void(btGeneric6DofConstraint_setAngularUpperLimit)(void*,const void*);
void(btWheelInfo_setSkidInfo)(void*,float);
void(btCollisionWorld_rayTestSingleInternal)(const void*,const void*,const void*,void*);
bool(btAABB_overlapping_trans_conservative2)(void*,const void*,const void*);
float(btMultiBodyConstraint_getPosition)(void*,int);
void*(btGImpactShapeInterface_getChildShape)(void*,int);
void*(btDefaultSerializer_new2)(int);
bool(btDiscreteDynamicsWorld_getLatencyMotionStateInterpolation)(void*);
void(btBroadphasePair_setPProxy1)(void*,void*);
void*(btWorldImporter_createGeneric6DofConstraint2)(void*,void*,const void*,bool);
bool(btGImpactShapeInterface_childrenHasTransform)(void*);
void(btManifoldPoint_setContactERP)(void*,float);
void(btDefaultCollisionConstructionInfo_setCustomCollisionAlgorithmMaxElementSize)(void*,int);
int(btDbvt_sStkNPS_getMask)(void*);
bool(btGeneric6DofConstraint_testAngularLimitMotor)(void*,int);
bool(btRotationalLimitMotor2_getSpringDampingLimited)(void*);
void*(btAABB_new)();
float(btDbvtBroadphase_getUpdates_ratio)(void*);
void(btTransformUtil_calculateVelocity)(const void*,const void*,float,void*,void*);
void(btGImpactBvh_getNodeBound)(void*,int,void*);
void*(btGeneric6DofConstraint_new)(void*,void*,const void*,const void*,bool);
void*(btCapsuleShape_new)(float,float);
void*(btGImpactQuantizedBvh_getPrimitiveManager)(void*);
void(btDefaultMotionState_setStartWorldTrans)(void*,const void*);
void(btTranslationalLimitMotor2_getUpperLimit)(void*,void*);
void*(btHingeAccumulatedAngleConstraint_new3)(void*,const void*,const void*);
void*(btGImpactCollisionAlgorithm_new)(const void*,const void*,const void*);
void(btCollisionObject_setCollisionFlags)(void*,int);
void*(btSoftBody_RContact_new)();
void(btDiscreteDynamicsWorld_updateVehicles)(void*,float);
bool(btSoftBody_checkContact)(void*,const void*,const void*,float,void*);
void(btTriangleMeshShape_localGetSupportingVertex)(void*,const void*,void*);
void(btGImpactCollisionAlgorithm_setPart1)(void*,int);
void(btGjkPairDetector_setLastUsedMethod)(void*,int);
void(btAABB_get_center_extend)(void*,void*,void*);
void(btConstraintSolver_allSolved)(void*,const void*,void*);
void(btDbvt_rayTestInternal)(void*,const void*,const void*,const void*,const void*,unsigned int*,float,const void*,const void*,const void*);
int(btDbvtBroadphase_getCupdates)(void*);
void(btTranslationalLimitMotor2_getServoTarget)(void*,void*);
void(btManifoldPoint_getLateralFrictionDir2)(void*,void*);
void(btAngularLimit_set)(void*,float,float);
void(btManifoldPoint_setNormalWorldOnB)(void*,const void*);
void(btMultibodyLink_setAppliedTorque)(void*,const void*);
int(btQuantizedBvhTree_getEscapeNodeIndex)(void*,int);
bool(btGImpactQuantizedBvh_boxQuery)(void*,const void*,void*);
void(btSimulationIslandManager_updateActivationState)(void*,void*,void*);
void(btGeneric6DofConstraint_getFrameOffsetA)(void*,void*);
void(btRotationalLimitMotor2_setTargetVelocity)(void*,float);
void(btGjkPairDetector_setMinkowskiA)(void*,const void*);
void(btDbvtBroadphase_setCupdates)(void*,int);
float(btConeTwistConstraint_getDamping)(void*);
float(btMultiBodyConstraintSolver_solveGroupCacheFriendlyFinish)(void*,void**,int,const void*);
void*(btGeneric6DofSpringConstraint_new)(void*,void*,const void*,const void*,bool);
float(btConeTwistConstraint_getSwingSpan2)(void*);
void(btGImpactQuantizedBvh_update)(void*);
void(btDynamicsWorld_removeConstraint)(void*,void*);
bool(btConvexCast_calcTimeOfImpact)(void*,const void*,const void*,const void*,const void*,void*);
void(btGeneric6DofSpring2Constraint_setTargetVelocity)(void*,int,float);
void*(btConvexConcaveCollisionAlgorithm_SwappedCreateFunc_new)();
float(btRotationalLimitMotor2_getStopERP)(void*);
void*(btGImpactBvh_new)();
void(btDbvt_setFree)(void*,void*);
float(btMultibodyLink_getJointDamping)(void*);
void(btRaycastVehicle_resetSuspension)(void*);
void(btGImpactShapeInterface_unlockChildShapes)(void*);
void*(btGImpactCompoundShape_CompoundPrimitiveManager_getCompoundShape)(void*);
void*(btCollisionWorld_getBroadphase)(void*);
void*(btSoftBody_new2)(void*);
void*(btGImpactMeshShapePart_TrimeshPrimitiveManager_new)(void*,int);
float(btRotationalLimitMotor_getDamping)(void*);
float(btSoftBody_SContact_getMargin)(void*);
int(btGImpactMeshShapePart_TrimeshPrimitiveManager_getNumverts)(void*);
void(btBvhTriangleMeshShape_serializeSingleBvh)(void*,void*);
void*(btDbvtNode_getData)(void*);
void(btStorageResult_setClosestPointInB)(void*,const void*);
void(btAxisSweep3_removeHandle)(void*,unsigned short,void*);
void(btGImpactMeshShapePart_TrimeshPrimitiveManager_setIndexstride)(void*,int);
void(btSoftBody_appendTetra2)(void*,int,int,int,int);
void(btGImpactMeshShapePart_TrimeshPrimitiveManager_get_vertex)(void*,unsigned int,void*);
void(btMultiBody_addJointTorque)(void*,int,float);
void(btHinge2Constraint_getAnchor2)(void*,void*);
void(btGImpactMeshShapePart_TrimeshPrimitiveManager_setScale)(void*,const void*);
void(btConvexPlaneCollisionAlgorithm_CreateFunc_setMinimumPointsPerturbationThreshold)(void*,int);
unsigned int(btCollisionWorld_RayResultCallback_getFlags)(void*);
int(btConvexTriangleCallback_getTriangleCount)(void*);
int(btAlignedObjectArray_btSoftBodyPtr_size)(void*);
void(btConvexInternalShape_setSafeMargin4)(void*,const void*,float);
void(btRotationalLimitMotor_setHiLimit)(void*,float);
void(btRotationalLimitMotor_setDamping)(void*,float);
void(btGeneric6DofSpring2Constraint_setLinearLowerLimit)(void*,const void*);
void(btGeneric6DofSpring2Constraint_setAngularLowerLimit)(void*,const void*);
int(btDbvt_countLeaves)(const void*);
void*(btCollisionDispatcher_new)(void*);
void(btInternalTriangleIndexCallback_delete)(void*);
void(btMultiBody_setJointVel)(void*,int,float);
void(btWorldImporter_setVerboseMode)(void*,int);
void(btGjkPairDetector_setCurIter)(void*,int);
int(btGImpactQuantizedBvh_getNodeData)(void*,int);
float(btBroadphaseRayCallback_getLambda_max)(void*);
void(btGeneric6DofSpring2Constraint_getAngularLowerLimit)(void*,void*);
void*(btHeightfieldTerrainShape_new)(int,int,const void*,float,float,float,int,int,bool);
float(btRigidBody_btRigidBodyConstructionInfo_getAdditionalLinearDampingThresholdSqr)(void*);
void(btDispatcherInfo_setStepCount)(void*,int);
void(btCollisionObject_setHitFraction)(void*,float);
void(btContactSolverInfoData_setRestitution)(void*,float);
void(btMultibodyLink_getAxisBottom)(void*,int,void*);
void*(btHingeConstraint_new2)(void*,void*,const void*,const void*,const void*,const void*,bool);
void*(btMultibodyLink_getJointFeedback)(void*);
void(btGImpactShapeInterface_rayTest)(void*,const void*,const void*,void*);
void(btDefaultCollisionConfiguration_setConvexConvexMultipointIterations3)(void*,int,int);
const void*(btManifoldResult_getBody1Internal)(void*);
float(btMultibodyLink_getMass)(void*);
void(btHingeConstraint_enableMotor)(void*,bool);
void*(btEmptyAlgorithm_CreateFunc_new)();
void(btHingeConstraint_getFrameOffsetA)(void*,void*);
float(btHingeConstraint_getHingeAngle2)(void*);
void*(btOverlappingPairCache_getOverlappingPairArrayPtr)(void*);
void(btMultiBody_getParentToLocalRot)(void*,int,void*);
int(btBvhTree_getNodeData)(void*,int);
void*(btKinematicCharacterController_new)(void*,void*,float);
bool(btBroadphaseProxy_isConvex2d)(int);
void(btMultibodyLink_getAppliedConstraintTorque)(void*,void*);
bool(btTranslationalLimitMotor_needApplyForce)(void*,int);
void(btHingeConstraint_getAFrame)(void*,void*);
void*(btConvexTriangleMeshShape_new2)(void*,bool);
int(btIndexedMesh_getVertexType)(void*);
float(btConeTwistConstraint_getTwistLimitSign)(void*);
void(btHingeConstraint_setAngularOnly)(void*,bool);
void(btSparseSdf_delete)(void*);
int(btSoftBody_Config_getAeromodel)(void*);
void(btJointFeedback_delete)(void*);
void(btGjkPairDetector_setIgnoreMargin)(void*,bool);
void(btRotationalLimitMotor_setCurrentLimitError)(void*,float);
void(btSoftBody_Joint_getMassmatrix)(void*,void*);
void(btMultiBodySolverConstraint_getRelpos1CrossNormal)(void*,void*);
int(btCollisionObject_getInternalType)(void*);
const void*(btConvex2dConvex2dAlgorithm_getManifold)(void*);
void(btSerializer_delete)(void*);
void*(btBulletWorldImporter_new)();
void(btQuantizedBvhTree_quantizePoint)(void*,unsigned short*,const void*);
void(btTranslationalLimitMotor2_setEquilibriumPoint)(void*,const void*);
void(btOptimizedBvhNode_getAabbMaxOrg)(void*,void*);
void(btKinematicCharacterController_setFallSpeed)(void*,float);
void(btMultiBody_applyDeltaVeeMultiDof2)(void*,const float*,float);
void(btSparseSdf3_GarbageCollect2)(void*);
void*(btConvexTriangleMeshShape_getMeshInterface)(void*);
float(btAngularLimit_getLow)(void*);
void(btRigidBody_getCenterOfMassTransform)(void*,void*);
void(btVoronoiSimplexSolver_delete)(void*);
void*(btCollisionDispatcher_getCollisionConfiguration)(void*);
void*(btDispatcher_findAlgorithm)(void*,const void*,const void*);
float(btManifoldPoint_getAppliedImpulseLateral2)(void*);
void(btDbvtBroadphase_setPaircache)(void*,void*);
float(btManifoldPoint_getCombinedRestitution)(void*);
float(btManifoldPoint_getContactCFM)(void*);
float(btManifoldPoint_getContactERP)(void*);
float(btManifoldPoint_getDistance)(void*);
void(btManifoldPoint_getLateralFrictionDir1)(void*,void*);
void*(btSoftBody_getCdbvt)(void*);
void*(btCollisionWorld_ClosestConvexResultCallback_new)(const void*,const void*);
void(btMultibodyLink_setAxisTop2)(void*,int,const void*);
void(btCollisionObject_setAnisotropicFriction2)(void*,const void*,int);
int(btCollisionShape_getShapeType)(void*);
void(btQuantizedBvh_setQuantizationValues)(void*,const void*,const void*);
void(btDbvtBroadphase_setFixedleft)(void*,int);
void(btConvexTriangleCallback_setTimeStepAndCounters)(void*,float,const void*,const void*,const void*,void*);
bool(btAABB_collide_plane)(void*,const void*);
void(btCompoundShapeChild_setChildShapeType)(void*,int);
void(btManifoldPoint_setIndex0)(void*,int);
void(btManifoldPoint_setLateralFrictionDir1)(void*,const void*);
void*(btCompoundFromGImpact_btCreateCompoundFromGimpactShape)(const void*,float);
void*(btDbvt_getStkStack)(void*);
float(btSoftBody_Cluster_getSelfCollisionImpulseFactor)(void*);
void*(btSoftBodyWorldInfo_new)();
void*(btWorldImporter_createCapsuleShapeZ)(void*,float,float);
void(btManifoldPoint_setCombinedRollingFriction)(void*,float);
void(btMultiBody_clearVelocities)(void*);
const void*(btManifoldResult_getBody1Wrap)(void*);
bool(btAABB_collide_triangle_exact)(void*,const void*,const void*,const void*,const void*);
void(btCollisionWorld_LocalConvexResult_setHitFraction)(void*,float);
void(btUnionFind_allocate)(void*,int);
bool(btTranslationalLimitMotor2_isLimited)(void*,int);
void(btRotationalLimitMotor_setBounce)(void*,float);
float(btWheelInfo_getSuspensionRestLength1)(void*);
void(btSoftBody_appendFace5)(void*,int,int,int,void*);
void*(btSoftBody_Impulse_operator_n)(void*);
void*(btTranslationalLimitMotor2_new)();
void(btManifoldResult_setBody0Wrap)(void*,const void*);
void(btMultiBody_addBaseForce)(void*,const void*);
void(btMultiBody_addBaseTorque)(void*,const void*);
void(btContactSolverInfoData_setRestingContactRestitutionThreshold)(void*,int);
bool(btGImpactBvh_boxQuery)(void*,const void*,void*);
void*(btRigidBody_btRigidBodyConstructionInfo_new2)(float,void*,void*,const void*);
void(btMultiBody_addLinkForce)(void*,int,const void*);
unsigned short(btAxisSweep3_getNumHandles)(void*);
int(btContactSolverInfoData_getNumIterations)(void*);
void(btGeneric6DofSpring2Constraint_getCalculatedTransformA)(void*,void*);
void(btTranslationalLimitMotor_setLimitSoftness)(void*,float);
float(btWheelInfo_getWheelsSuspensionForce)(void*);
void(btConvexPolyhedron_project)(void*,const void*,const void*,float*,float*,void*,void*);
void(btMultiBody_getBaseWorldTransform)(void*,void*);
void*(btAxisSweep3_new)(const void*,const void*);
float*(btMultiBody_getJointPosMultiDof)(void*,int);
void(btMultiBody_goToSleep)(void*);
void(btGeneric6DofSpring2Constraint_getLinearLowerLimit)(void*,void*);
bool(btBulletWorldImporter_loadFileFromMemory2)(void*,void*);
float(btMultiBody_getLinkMass)(void*,int);
int(btMultiBodyConstraint_getNumRows)(void*);
int(btDbvt_sStkNP_getMask)(void*);
void(btGImpactMeshShapePart_TrimeshPrimitiveManager_setMargin)(void*,float);
void(btMultiBody_localDirToWorld)(void*,int,const void*,void*);
void(btConvexTriangleCallback_clearWrapperData)(void*);
void(btMultiBody_setJointPosMultiDof)(void*,int,float*);
void(btIndexedMesh_setVertexBase)(void*,const unsigned char*);
void(btMultibodyLink_getEVector)(void*,void*);
void(btMultiBody_setupPlanar)(void*,int,float,const void*,int,const void*,const void*,const void*);
void(btMultiBody_setupRevolute2)(void*,int,float,const void*,int,const void*,const void*,const void*,const void*,bool);
void*(btCompoundShape_new3)(bool,int);
void(btWheelInfo_setBIsFrontWheel)(void*,bool);
void(btAlignedObjectArray_btCollisionObjectPtr_resizeNoInitialize)(void*,int);
void(btSphereShape_setUnscaledRadius)(void*,float);
int(btMultiBody_getNumLinks)(void*);
void(btPoint2PointConstraint_setPivotB)(void*,const void*);
void(btGeneric6DofConstraint_calculateTransforms)(void*,const void*,const void*);
void(btMultibodyLink_getAppliedConstraintForce)(void*,void*);
void(btMultibodyLink_getAxisTop)(void*,int,void*);
void(btMultibodyLink_getCachedRotParentToThis)(void*,void*);
void(btMultibodyLink_getCachedWorldTransform)(void*,void*);
int(btMultibodyLink_getCfgOffset)(void*);
void(btRotationalLimitMotor2_setEnableSpring)(void*,bool);
int(btMultibodyLink_getDofOffset)(void*);
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
	RotationalLimitMotor_getLimitSoftness = CLIB.btRotationalLimitMotor_getLimitSoftness,
	RigidBody_computeGyroscopicImpulseImplicit_World = CLIB.btRigidBody_computeGyroscopicImpulseImplicit_World,
	WorldImporter_createGearConstraint = CLIB.btWorldImporter_createGearConstraint,
	RotationalLimitMotor2_getSpringStiffnessLimited = CLIB.btRotationalLimitMotor2_getSpringStiffnessLimited,
	GImpactCollisionAlgorithm_gimpact_vs_concave = CLIB.btGImpactCollisionAlgorithm_gimpact_vs_concave,
	TriangleIndexVertexArray_getIndexedMeshArray = CLIB.btTriangleIndexVertexArray_getIndexedMeshArray,
	CollisionConfiguration_delete = CLIB.btCollisionConfiguration_delete,
	CollisionWorld_LocalConvexResult_getHitCollisionObject = CLIB.btCollisionWorld_LocalConvexResult_getHitCollisionObject,
	Generic6DofSpring2Constraint_setStiffness2 = CLIB.btGeneric6DofSpring2Constraint_setStiffness2,
	CollisionWorld_LocalConvexResult_delete = CLIB.btCollisionWorld_LocalConvexResult_delete,
	PrimitiveManagerBase_get_primitive_triangle = CLIB.btPrimitiveManagerBase_get_primitive_triangle,
	ContinuousConvexCollision_new = CLIB.btContinuousConvexCollision_new,
	CollisionObject_isActive = CLIB.btCollisionObject_isActive,
	RotationalLimitMotor2_getCurrentLimit = CLIB.btRotationalLimitMotor2_getCurrentLimit,
	RaycastVehicle_getSteeringValue = CLIB.btRaycastVehicle_getSteeringValue,
	JointFeedback_getAppliedTorqueBodyB = CLIB.btJointFeedback_getAppliedTorqueBodyB,
	RotationalLimitMotor2_getServoTarget = CLIB.btRotationalLimitMotor2_getServoTarget,
	GImpactCollisionAlgorithm_internalGetResultOut = CLIB.btGImpactCollisionAlgorithm_internalGetResultOut,
	Dbvt_sStkNPS_delete = CLIB.btDbvt_sStkNPS_delete,
	BroadphaseInterface_rayTest2 = CLIB.btBroadphaseInterface_rayTest2,
	HingeConstraint_getInfo2InternalUsingFrameOffset = CLIB.btHingeConstraint_getInfo2InternalUsingFrameOffset,
	CollisionWorld_LocalRayResult_getHitNormalLocal = CLIB.btCollisionWorld_LocalRayResult_getHitNormalLocal,
	ManifoldPoint_setAppliedImpulseLateral1 = CLIB.btManifoldPoint_setAppliedImpulseLateral1,
	SoftBody_getTetraVertexNormalData2 = CLIB.btSoftBody_getTetraVertexNormalData2,
	CollisionWorld_convexSweepTest = CLIB.btCollisionWorld_convexSweepTest,
	MLCPSolver_setMLCPSolver = CLIB.btMLCPSolver_setMLCPSolver,
	OverlappingPairCache_setInternalGhostPairCallback = CLIB.btOverlappingPairCache_setInternalGhostPairCallback,
	AlignedObjectArray_btSoftBody_JointPtr_resizeNoInitialize = CLIB.btAlignedObjectArray_btSoftBody_JointPtr_resizeNoInitialize,
	UnionFind_new = CLIB.btUnionFind_new,
	MultiBodySolverConstraint_getJacAindex = CLIB.btMultiBodySolverConstraint_getJacAindex,
	MultiSphereShape_getSphereRadius = CLIB.btMultiSphereShape_getSphereRadius,
	TranslationalLimitMotor_getAccumulatedImpulse = CLIB.btTranslationalLimitMotor_getAccumulatedImpulse,
	MultiBody_setBaseInertia = CLIB.btMultiBody_setBaseInertia,
	CollisionShape_isInfinite = CLIB.btCollisionShape_isInfinite,
	ConeTwistConstraint_new2 = CLIB.btConeTwistConstraint_new2,
	RaycastVehicle_setPitchControl = CLIB.btRaycastVehicle_setPitchControl,
	TriangleIndexVertexArray_new = CLIB.btTriangleIndexVertexArray_new,
	Dbvt_clone = CLIB.btDbvt_clone,
	StridingMeshInterface_unLockReadOnlyVertexBase = CLIB.btStridingMeshInterface_unLockReadOnlyVertexBase,
	TetrahedronShapeEx_setVertices = CLIB.btTetrahedronShapeEx_setVertices,
	ContactSolverInfoData_getSingleAxisRollingFrictionThreshold = CLIB.btContactSolverInfoData_getSingleAxisRollingFrictionThreshold,
	CollisionShape_setUserPointer = CLIB.btCollisionShape_setUserPointer,
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
	BroadphaseProxy_getClientObject = CLIB.btBroadphaseProxy_getClientObject,
	SoftBody_getTotalMass = CLIB.btSoftBody_getTotalMass,
	DispatcherInfo_setTimeOfImpact = CLIB.btDispatcherInfo_setTimeOfImpact,
	SoftBody_Config_getKSK_SPLT_CL = CLIB.btSoftBody_Config_getKSK_SPLT_CL,
	SoftBody_Config_getMaxvolume = CLIB.btSoftBody_Config_getMaxvolume,
	MotionStateWrapper_new = CLIB.btMotionStateWrapper_new,
	RaycastVehicle_getForwardVector = CLIB.btRaycastVehicle_getForwardVector,
	AlignedObjectArray_btCollisionObjectPtr_at = CLIB.btAlignedObjectArray_btCollisionObjectPtr_at,
	ManifoldPoint_getUserPersistentData = CLIB.btManifoldPoint_getUserPersistentData,
	RotationalLimitMotor2_new = CLIB.btRotationalLimitMotor2_new,
	RotationalLimitMotor2_setSpringDamping = CLIB.btRotationalLimitMotor2_setSpringDamping,
	TranslationalLimitMotor2_getEquilibriumPoint = CLIB.btTranslationalLimitMotor2_getEquilibriumPoint,
	ConvexTriangleCallback_getAabbMax = CLIB.btConvexTriangleCallback_getAabbMax,
	HeightfieldTerrainShape_setUseDiamondSubdivision2 = CLIB.btHeightfieldTerrainShape_setUseDiamondSubdivision2,
	HingeConstraint_setLimit2 = CLIB.btHingeConstraint_setLimit2,
	AngularLimit_set3 = CLIB.btAngularLimit_set3,
	SoftBody_AJoint_IControl_Speed = CLIB.btSoftBody_AJoint_IControl_Speed,
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
	CharacterControllerInterface_setUpInterpolate = CLIB.btCharacterControllerInterface_setUpInterpolate,
	CollisionWorld_AllHitsRayResultCallback_getCollisionObjects = CLIB.btCollisionWorld_AllHitsRayResultCallback_getCollisionObjects,
	ConvexShape_getNumPreferredPenetrationDirections = CLIB.btConvexShape_getNumPreferredPenetrationDirections,
	DefaultCollisionConfiguration_setPlaneConvexMultipointIterations2 = CLIB.btDefaultCollisionConfiguration_setPlaneConvexMultipointIterations2,
	SoftBody_appendFace4 = CLIB.btSoftBody_appendFace4,
	SoftBody_Impulse_operator_m = CLIB.btSoftBody_Impulse_operator_m,
	ConvexPolyhedron_initialize = CLIB.btConvexPolyhedron_initialize,
	DbvtAabbMm_Maxs = CLIB.btDbvtAabbMm_Maxs,
	SliderConstraint_getFlags = CLIB.btSliderConstraint_getFlags,
	MultiBodyJointMotor_new = CLIB.btMultiBodyJointMotor_new,
	BroadphaseProxy_getMultiSapParentProxy = CLIB.btBroadphaseProxy_getMultiSapParentProxy,
	SimulationIslandManager_IslandCallback_delete = CLIB.btSimulationIslandManager_IslandCallback_delete,
	CapsuleShape_deSerializeFloat = CLIB.btCapsuleShape_deSerializeFloat,
	SoftBody_Node_setN = CLIB.btSoftBody_Node_setN,
	CollisionWorld_delete = CLIB.btCollisionWorld_delete,
	HingeAccumulatedAngleConstraint_new6 = CLIB.btHingeAccumulatedAngleConstraint_new6,
	Dbvt_update3 = CLIB.btDbvt_update3,
	ConvexPolyhedron_new = CLIB.btConvexPolyhedron_new,
	BvhTriangleMeshShape_serializeSingleTriangleInfoMap = CLIB.btBvhTriangleMeshShape_serializeSingleTriangleInfoMap,
	GImpactQuantizedBvh_get_node_pointer2 = CLIB.btGImpactQuantizedBvh_get_node_pointer2,
	TriangleShapeEx_buildTriPlane = CLIB.btTriangleShapeEx_buildTriPlane,
	HingeConstraint_getSolveLimit = CLIB.btHingeConstraint_getSolveLimit,
	Dbvt_getFree = CLIB.btDbvt_getFree,
	CollisionObject_setAnisotropicFriction = CLIB.btCollisionObject_setAnisotropicFriction,
	CompoundShape_new2 = CLIB.btCompoundShape_new2,
	Point2PointConstraint_setPivotA = CLIB.btPoint2PointConstraint_setPivotA,
	DbvtAabbMm_new = CLIB.btDbvtAabbMm_new,
	Generic6DofSpring2Constraint_new4 = CLIB.btGeneric6DofSpring2Constraint_new4,
	ConvexPointCloudShape_new = CLIB.btConvexPointCloudShape_new,
	BroadphaseInterface_createProxy = CLIB.btBroadphaseInterface_createProxy,
	ContactSolverInfoData_getFriction = CLIB.btContactSolverInfoData_getFriction,
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
	DispatcherInfo_setTimeStep = CLIB.btDispatcherInfo_setTimeStep,
	BvhTree_getNodeBound = CLIB.btBvhTree_getNodeBound,
	PersistentManifold_addManifoldPoint2 = CLIB.btPersistentManifold_addManifoldPoint2,
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
	WheelInfo_getWheelAxleCS = CLIB.btWheelInfo_getWheelAxleCS,
	DynamicsWorld_addConstraint = CLIB.btDynamicsWorld_addConstraint,
	GImpactCollisionAlgorithm_getFace1 = CLIB.btGImpactCollisionAlgorithm_getFace1,
	CollisionWorld_ConvexResultCallback_setCollisionFilterMask = CLIB.btCollisionWorld_ConvexResultCallback_setCollisionFilterMask,
	MultiBody_getCanSleep = CLIB.btMultiBody_getCanSleep,
	SliderConstraint_getSoftnessOrthoAng = CLIB.btSliderConstraint_getSoftnessOrthoAng,
	TypedConstraint_getUid = CLIB.btTypedConstraint_getUid,
	MultiBodyConstraintSolver_solveMultiBodyGroup = CLIB.btMultiBodyConstraintSolver_solveMultiBodyGroup,
	Generic6DofConstraint_getInfo1NonVirtual = CLIB.btGeneric6DofConstraint_getInfo1NonVirtual,
	Chunk_new = CLIB.btChunk_new,
	WheelInfoConstructionInfo_setWheelsDampingRelaxation = CLIB.btWheelInfoConstructionInfo_setWheelsDampingRelaxation,
	Dbvt_getOpath = CLIB.btDbvt_getOpath,
	BU_Simplex1to4_getIndex = CLIB.btBU_Simplex1to4_getIndex,
	AABB_projection_interval = CLIB.btAABB_projection_interval,
	ContactSolverInfoData_setSolverMode = CLIB.btContactSolverInfoData_setSolverMode,
	SubSimplexClosestResult_setBarycentricCoordinates4 = CLIB.btSubSimplexClosestResult_setBarycentricCoordinates4,
	RigidBody_btRigidBodyConstructionInfo_setAdditionalLinearDampingThresholdSqr = CLIB.btRigidBody_btRigidBodyConstructionInfo_setAdditionalLinearDampingThresholdSqr,
	RaycastVehicle_setBrake = CLIB.btRaycastVehicle_setBrake,
	TranslationalLimitMotor_solveLinearAxis = CLIB.btTranslationalLimitMotor_solveLinearAxis,
	RigidBody_btRigidBodyConstructionInfo_getAdditionalAngularDampingThresholdSqr = CLIB.btRigidBody_btRigidBodyConstructionInfo_getAdditionalAngularDampingThresholdSqr,
	DefaultMotionState_setGraphicsWorldTrans = CLIB.btDefaultMotionState_setGraphicsWorldTrans,
	ConeTwistConstraint_getRelaxationFactor = CLIB.btConeTwistConstraint_getRelaxationFactor,
	Generic6DofSpring2Constraint_setLinearUpperLimit = CLIB.btGeneric6DofSpring2Constraint_setLinearUpperLimit,
	SoftBody_SContact_getFace = CLIB.btSoftBody_SContact_getFace,
	SoftBody_Cluster_setContainsAnchor = CLIB.btSoftBody_Cluster_setContainsAnchor,
	UsageBitfield_getUnused3 = CLIB.btUsageBitfield_getUnused3,
	Dbvt_rayTest = CLIB.btDbvt_rayTest,
	ContactSolverInfoData_setTimeStep = CLIB.btContactSolverInfoData_setTimeStep,
	QuantizedBvh_reportRayOverlappingNodex = CLIB.btQuantizedBvh_reportRayOverlappingNodex,
	CollisionObject_getCollisionFlags = CLIB.btCollisionObject_getCollisionFlags,
	DiscreteDynamicsWorld_synchronizeSingleMotionState = CLIB.btDiscreteDynamicsWorld_synchronizeSingleMotionState,
	OverlappingPairCache_hasDeferredRemoval = CLIB.btOverlappingPairCache_hasDeferredRemoval,
	CollisionWorld_RayResultCallback_getCollisionFilterGroup = CLIB.btCollisionWorld_RayResultCallback_getCollisionFilterGroup,
	BroadphaseProxy_setAabbMax = CLIB.btBroadphaseProxy_setAabbMax,
	MultiBody_wakeUp = CLIB.btMultiBody_wakeUp,
	SoftBody_RContact_getCti = CLIB.btSoftBody_RContact_getCti,
	Dbvt_IClone_CloneLeaf = CLIB.btDbvt_IClone_CloneLeaf,
	RaycastVehicle_setCoordinateSystem = CLIB.btRaycastVehicle_setCoordinateSystem,
	UnionFind_delete = CLIB.btUnionFind_delete,
	CollisionWorld_setBroadphase = CLIB.btCollisionWorld_setBroadphase,
	WorldImporter_createCapsuleShapeY = CLIB.btWorldImporter_createCapsuleShapeY,
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
	SoftBody_appendAnchor4 = CLIB.btSoftBody_appendAnchor4,
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
	Dbvt_insert = CLIB.btDbvt_insert,
	NNCGConstraintSolver_setOnlyForNoneContact = CLIB.btNNCGConstraintSolver_setOnlyForNoneContact,
	Dbvt_sStkNP_new = CLIB.btDbvt_sStkNP_new,
	AlignedObjectArray_btSoftBody_Tetra_resizeNoInitialize = CLIB.btAlignedObjectArray_btSoftBody_Tetra_resizeNoInitialize,
	SubSimplexClosestResult_setBarycentricCoordinates3 = CLIB.btSubSimplexClosestResult_setBarycentricCoordinates3,
	QuantizedBvh_delete = CLIB.btQuantizedBvh_delete,
	RaycastVehicle_getCurrentSpeedKmHour = CLIB.btRaycastVehicle_getCurrentSpeedKmHour,
	Generic6DofConstraint_setAxis = CLIB.btGeneric6DofConstraint_setAxis,
	DiscreteCollisionDetectorInterface_ClosestPointInput_setTransformB = CLIB.btDiscreteCollisionDetectorInterface_ClosestPointInput_setTransformB,
	CollisionShape_serialize = CLIB.btCollisionShape_serialize,
	CompoundCollisionAlgorithm_SwappedCreateFunc_new = CLIB.btCompoundCollisionAlgorithm_SwappedCreateFunc_new,
	CollisionWorld_ClosestConvexResultCallback_setHitCollisionObject = CLIB.btCollisionWorld_ClosestConvexResultCallback_setHitCollisionObject,
	CollisionWorld_rayTest = CLIB.btCollisionWorld_rayTest,
	ConvexPolyhedron_getVertices = CLIB.btConvexPolyhedron_getVertices,
	GImpactQuantizedBvh_getEscapeNodeIndex = CLIB.btGImpactQuantizedBvh_getEscapeNodeIndex,
	ConvexConvexAlgorithm_CreateFunc_getSimplexSolver = CLIB.btConvexConvexAlgorithm_CreateFunc_getSimplexSolver,
	CylinderShape_new = CLIB.btCylinderShape_new,
	SoftBody_Impulse_getAsDrift = CLIB.btSoftBody_Impulse_getAsDrift,
	StridingMeshInterface_calculateAabbBruteForce = CLIB.btStridingMeshInterface_calculateAabbBruteForce,
	MultiBodySolverConstraint_setAngularComponentB = CLIB.btMultiBodySolverConstraint_setAngularComponentB,
	ContactSolverInfoData_setTau = CLIB.btContactSolverInfoData_setTau,
	WorldImporter_createConeShapeY = CLIB.btWorldImporter_createConeShapeY,
	TriangleInfoMap_getMaxEdgeAngleThreshold = CLIB.btTriangleInfoMap_getMaxEdgeAngleThreshold,
	RotationalLimitMotor_setAccumulatedImpulse = CLIB.btRotationalLimitMotor_setAccumulatedImpulse,
	ConvexHullShape_new2 = CLIB.btConvexHullShape_new2,
	SubSimplexClosestResult_isValid = CLIB.btSubSimplexClosestResult_isValid,
	RigidBody_new = CLIB.btRigidBody_new,
	RaycastVehicle_getUpAxis = CLIB.btRaycastVehicle_getUpAxis,
	AngularLimit_getBiasFactor = CLIB.btAngularLimit_getBiasFactor,
	RigidBody_btRigidBodyConstructionInfo_getAdditionalAngularDampingFactor = CLIB.btRigidBody_btRigidBodyConstructionInfo_getAdditionalAngularDampingFactor,
	SliderConstraint_getInfo2NonVirtual = CLIB.btSliderConstraint_getInfo2NonVirtual,
	BroadphaseProxy_getAabbMin = CLIB.btBroadphaseProxy_getAabbMin,
	AxisSweep3_addHandle = CLIB.btAxisSweep3_addHandle,
	OverlappingPairCache_getOverlappingPairArray = CLIB.btOverlappingPairCache_getOverlappingPairArray,
	OverlappingPairCallback_delete = CLIB.btOverlappingPairCallback_delete,
	MultiBody_getBaseCollider = CLIB.btMultiBody_getBaseCollider,
	SoftBody_CJoint_getMaxlife = CLIB.btSoftBody_CJoint_getMaxlife,
	MultiBodyJointMotor_new2 = CLIB.btMultiBodyJointMotor_new2,
	MultibodyLink_setCfgOffset = CLIB.btMultibodyLink_setCfgOffset,
	SoftBody_Cluster_getIdmass = CLIB.btSoftBody_Cluster_getIdmass,
	BvhTriangleMeshShape_getOptimizedBvh = CLIB.btBvhTriangleMeshShape_getOptimizedBvh,
	IDebugDrawWrapper_getGCHandle = CLIB.btIDebugDrawWrapper_getGCHandle,
	SoftBody_Config_getKDG = CLIB.btSoftBody_Config_getKDG,
	GImpactBvh_getPrimitiveManager = CLIB.btGImpactBvh_getPrimitiveManager,
	GImpactMeshShapePart_TrimeshPrimitiveManager_getIndexbase = CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_getIndexbase,
	GImpactCollisionAlgorithm_setFace1 = CLIB.btGImpactCollisionAlgorithm_setFace1,
	TypedConstraint_btConstraintInfo1_setNumConstraintRows = CLIB.btTypedConstraint_btConstraintInfo1_setNumConstraintRows,
	Box2dShape_getPlaneEquation = CLIB.btBox2dShape_getPlaneEquation,
	Point2PointConstraint_setUseSolveConstraintObsolete = CLIB.btPoint2PointConstraint_setUseSolveConstraintObsolete,
	DynamicsWorld_setInternalTickCallback3 = CLIB.btDynamicsWorld_setInternalTickCallback3,
	GImpactMeshShapePart_TrimeshPrimitiveManager_getIndicestype = CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_getIndicestype,
	UniversalConstraint_getAngle1 = CLIB.btUniversalConstraint_getAngle1,
	BroadphaseProxy_setClientObject = CLIB.btBroadphaseProxy_setClientObject,
	DiscreteCollisionDetectorInterface_getClosestPoints2 = CLIB.btDiscreteCollisionDetectorInterface_getClosestPoints2,
	SoftRigidDynamicsWorld_removeSoftBody = CLIB.btSoftRigidDynamicsWorld_removeSoftBody,
	GjkPairDetector_getCurIter = CLIB.btGjkPairDetector_getCurIter,
	SoftBody_appendLink7 = CLIB.btSoftBody_appendLink7,
	ManifoldPoint_delete = CLIB.btManifoldPoint_delete,
	AABB_collide_ray = CLIB.btAABB_collide_ray,
	SoftBody_appendNote = CLIB.btSoftBody_appendNote,
	GjkConvexCast_new = CLIB.btGjkConvexCast_new,
	KinematicCharacterController_setMaxJumpHeight = CLIB.btKinematicCharacterController_setMaxJumpHeight,
	HingeConstraint_new8 = CLIB.btHingeConstraint_new8,
	TypedConstraint_delete = CLIB.btTypedConstraint_delete,
	TriangleBuffer_getTriangle = CLIB.btTriangleBuffer_getTriangle,
	ManifoldPoint_setFrictionCFM = CLIB.btManifoldPoint_setFrictionCFM,
	DbvtBroadphase_getUpdates_done = CLIB.btDbvtBroadphase_getUpdates_done,
	ManifoldPoint_getCombinedFriction = CLIB.btManifoldPoint_getCombinedFriction,
	ConstraintSolver_reset = CLIB.btConstraintSolver_reset,
	ConeTwistConstraint_getSwingSpan1 = CLIB.btConeTwistConstraint_getSwingSpan1,
	AABB_copy_with_margin = CLIB.btAABB_copy_with_margin,
	DiscreteCollisionDetectorInterface_ClosestPointInput_getTransformA = CLIB.btDiscreteCollisionDetectorInterface_ClosestPointInput_getTransformA,
	MultiBodySolverConstraint_setLowerLimit = CLIB.btMultiBodySolverConstraint_setLowerLimit,
	CollisionWorld_ConvexResultCallback_needsCollision = CLIB.btCollisionWorld_ConvexResultCallback_needsCollision,
	MultiBodySolverConstraint_setFriction = CLIB.btMultiBodySolverConstraint_setFriction,
	RigidBody_btRigidBodyConstructionInfo_delete = CLIB.btRigidBody_btRigidBodyConstructionInfo_delete,
	AlignedObjectArray_btVector3_delete = CLIB.btAlignedObjectArray_btVector3_delete,
	AlignedObjectArray_btSoftBody_MaterialPtr_size = CLIB.btAlignedObjectArray_btSoftBody_MaterialPtr_size,
	UnionFind_Free = CLIB.btUnionFind_Free,
	AlignedObjectArray_btVector3_size = CLIB.btAlignedObjectArray_btVector3_size,
	SoftBody_sRayCast_getFeature = CLIB.btSoftBody_sRayCast_getFeature,
	WorldImporter_createSliderConstraint2 = CLIB.btWorldImporter_createSliderConstraint2,
	WorldImporter_createMultiSphereShape = CLIB.btWorldImporter_createMultiSphereShape,
	RotationalLimitMotor2_setHiLimit = CLIB.btRotationalLimitMotor2_setHiLimit,
	MultibodyLink_setCachedWorldTransform = CLIB.btMultibodyLink_setCachedWorldTransform,
	AlignedObjectArray_btVector3_set = CLIB.btAlignedObjectArray_btVector3_set,
	MultiBody_setupSpherical = CLIB.btMultiBody_setupSpherical,
	SoftBody_Link_setC1 = CLIB.btSoftBody_Link_setC1,
	AlignedObjectArray_btVector3_push_back = CLIB.btAlignedObjectArray_btVector3_push_back,
	Dbvt_update = CLIB.btDbvt_update,
	SliderConstraint_setRestitutionDirLin = CLIB.btSliderConstraint_setRestitutionDirLin,
	MultiBody_getUseGyroTerm = CLIB.btMultiBody_getUseGyroTerm,
	Generic6DofSpring2Constraint_getRotationalLimitMotor = CLIB.btGeneric6DofSpring2Constraint_getRotationalLimitMotor,
	ConvexCast_CastResult_new = CLIB.btConvexCast_CastResult_new,
	DynamicsWorld_addAction = CLIB.btDynamicsWorld_addAction,
	AlignedObjectArray_btVector3_new = CLIB.btAlignedObjectArray_btVector3_new,
	ConvexInternalShape_setImplicitShapeDimensions = CLIB.btConvexInternalShape_setImplicitShapeDimensions,
	SphereSphereCollisionAlgorithm_new2 = CLIB.btSphereSphereCollisionAlgorithm_new2,
	SoftBody_Config_getKVCF = CLIB.btSoftBody_Config_getKVCF,
	Vector3_array_at = CLIB.btVector3_array_at,
	SoftBodyNodePtrArray_set = CLIB.btSoftBodyNodePtrArray_set,
	GImpactQuantizedBvh_isLeafNode = CLIB.btGImpactQuantizedBvh_isLeafNode,
	SoftBodyNodePtrArray_at = CLIB.btSoftBodyNodePtrArray_at,
	WheelInfo_RaycastInfo_getGroundObject = CLIB.btWheelInfo_RaycastInfo_getGroundObject,
	DbvtBroadphase_getPaircache = CLIB.btDbvtBroadphase_getPaircache,
	RigidBody_getOrientation = CLIB.btRigidBody_getOrientation,
	WorldImporter_delete = CLIB.btWorldImporter_delete,
	CollisionObject_activate = CLIB.btCollisionObject_activate,
	DbvtNode_isleaf = CLIB.btDbvtNode_isleaf,
	PersistentManifold_setIndex1a = CLIB.btPersistentManifold_setIndex1a,
	RotationalLimitMotor2_delete = CLIB.btRotationalLimitMotor2_delete,
	WorldImporter_getVerboseMode = CLIB.btWorldImporter_getVerboseMode,
	SoftBody_appendAngularJoint3 = CLIB.btSoftBody_appendAngularJoint3,
	MultiBodySolverConstraint_delete = CLIB.btMultiBodySolverConstraint_delete,
	DefaultCollisionConstructionInfo_getDefaultMaxPersistentManifoldPoolSize = CLIB.btDefaultCollisionConstructionInfo_getDefaultMaxPersistentManifoldPoolSize,
	TriangleIndexVertexMaterialArray_addMaterialProperties2 = CLIB.btTriangleIndexVertexMaterialArray_addMaterialProperties2,
	SoftBodySolver_delete = CLIB.btSoftBodySolver_delete,
	MultiBody_getJointTorqueMultiDof = CLIB.btMultiBody_getJointTorqueMultiDof,
	SoftBody_Config_getKAHR = CLIB.btSoftBody_Config_getKAHR,
	WorldImporter_getRigidBodyByIndex = CLIB.btWorldImporter_getRigidBodyByIndex,
	CollisionWorld_convexSweepTest2 = CLIB.btCollisionWorld_convexSweepTest2,
	WorldImporter_getNumTriangleInfoMaps = CLIB.btWorldImporter_getNumTriangleInfoMaps,
	WorldImporter_new = CLIB.btWorldImporter_new,
	TranslationalLimitMotor_getDamping = CLIB.btTranslationalLimitMotor_getDamping,
	SoftBody_Cluster_setLocii = CLIB.btSoftBody_Cluster_setLocii,
	TriangleInfo_setFlags = CLIB.btTriangleInfo_setFlags,
	WorldImporter_getNumBvhs = CLIB.btWorldImporter_getNumBvhs,
	SoftBody_getLinks = CLIB.btSoftBody_getLinks,
	AxisSweep3_new4 = CLIB.btAxisSweep3_new4,
	SoftBody_Body_angularVelocity2 = CLIB.btSoftBody_Body_angularVelocity2,
	WorldImporter_getConstraintByName = CLIB.btWorldImporter_getConstraintByName,
	BroadphasePair_new = CLIB.btBroadphasePair_new,
	TranslationalLimitMotor_getNormalCFM = CLIB.btTranslationalLimitMotor_getNormalCFM,
	SoftBody_Cluster_setImass = CLIB.btSoftBody_Cluster_setImass,
	WorldImporter_getCollisionShapeByName = CLIB.btWorldImporter_getCollisionShapeByName,
	MinkowskiPenetrationDepthSolver_new = CLIB.btMinkowskiPenetrationDepthSolver_new,
	BU_Simplex1to4_new4 = CLIB.btBU_Simplex1to4_new4,
	CollisionObject_isKinematicObject = CLIB.btCollisionObject_isKinematicObject,
	GImpactShapeInterface_lockChildShapes = CLIB.btGImpactShapeInterface_lockChildShapes,
	BroadphaseProxy_setUniqueId = CLIB.btBroadphaseProxy_setUniqueId,
	SliderConstraint_setRestitutionOrthoLin = CLIB.btSliderConstraint_setRestitutionOrthoLin,
	StridingMeshInterface_getPremadeAabb = CLIB.btStridingMeshInterface_getPremadeAabb,
	SparseSdf3_GarbageCollect = CLIB.btSparseSdf3_GarbageCollect,
	SliderConstraint_getAngularPos = CLIB.btSliderConstraint_getAngularPos,
	ManifoldPoint_getAppliedImpulse = CLIB.btManifoldPoint_getAppliedImpulse,
	WorldImporter_createCapsuleShapeX = CLIB.btWorldImporter_createCapsuleShapeX,
	ContactSolverInfo_new = CLIB.btContactSolverInfo_new,
	OptimizedBvhNode_getTriangleIndex = CLIB.btOptimizedBvhNode_getTriangleIndex,
	ConeTwistConstraint_setFixThresh = CLIB.btConeTwistConstraint_setFixThresh,
	SoftBody_Body_applyVAImpulse = CLIB.btSoftBody_Body_applyVAImpulse,
	WorldImporter_createTriangleMeshContainer = CLIB.btWorldImporter_createTriangleMeshContainer,
	CollisionObject_serializeSingleObject = CLIB.btCollisionObject_serializeSingleObject,
	CollisionWorld_ClosestConvexResultCallback_getHitCollisionObject = CLIB.btCollisionWorld_ClosestConvexResultCallback_getHitCollisionObject,
	DbvtBroadphase_setGid = CLIB.btDbvtBroadphase_setGid,
	WorldImporter_createStridingMeshInterfaceData = CLIB.btWorldImporter_createStridingMeshInterfaceData,
	WorldImporter_createSphereShape = CLIB.btWorldImporter_createSphereShape,
	SoftBody_appendNote8 = CLIB.btSoftBody_appendNote8,
	WorldImporter_createSliderConstraint = CLIB.btWorldImporter_createSliderConstraint,
	TypedConstraint_btConstraintInfo2_setConstraintError = CLIB.btTypedConstraint_btConstraintInfo2_setConstraintError,
	CollisionWorld_ConvexResultCallback_setClosestHitFraction = CLIB.btCollisionWorld_ConvexResultCallback_setClosestHitFraction,
	HeightfieldTerrainShape_setUseZigzagSubdivision2 = CLIB.btHeightfieldTerrainShape_setUseZigzagSubdivision2,
	MultiBody_setHasSelfCollision = CLIB.btMultiBody_setHasSelfCollision,
	SoftBody_Link_getC0 = CLIB.btSoftBody_Link_getC0,
	SoftBody_Link_setC0 = CLIB.btSoftBody_Link_setC0,
	Hinge2Constraint_getAxis2 = CLIB.btHinge2Constraint_getAxis2,
	SliderConstraint_setUseFrameOffset = CLIB.btSliderConstraint_setUseFrameOffset,
	CollisionWorld_RayResultCallback_getClosestHitFraction = CLIB.btCollisionWorld_RayResultCallback_getClosestHitFraction,
	WorldImporter_createPlaneShape = CLIB.btWorldImporter_createPlaneShape,
	SoftBody_getFaceVertexData = CLIB.btSoftBody_getFaceVertexData,
	WorldImporter_createOptimizedBvh = CLIB.btWorldImporter_createOptimizedBvh,
	WorldImporter_createMeshInterface = CLIB.btWorldImporter_createMeshInterface,
	WorldImporter_createHingeConstraint4 = CLIB.btWorldImporter_createHingeConstraint4,
	MaterialProperties_getMaterialType = CLIB.btMaterialProperties_getMaterialType,
	WorldImporter_createHingeConstraint3 = CLIB.btWorldImporter_createHingeConstraint3,
	WorldImporter_createHingeConstraint2 = CLIB.btWorldImporter_createHingeConstraint2,
	WorldImporter_createHingeConstraint = CLIB.btWorldImporter_createHingeConstraint,
	QuantizedBvh_buildInternal = CLIB.btQuantizedBvh_buildInternal,
	UsageBitfield_setUnused1 = CLIB.btUsageBitfield_setUnused1,
	AlignedObjectArray_btSoftBody_ClusterPtr_push_back = CLIB.btAlignedObjectArray_btSoftBody_ClusterPtr_push_back,
	WorldImporter_createGeneric6DofSpring2Constraint = CLIB.btWorldImporter_createGeneric6DofSpring2Constraint,
	SoftBodyHelpers_Draw = CLIB.btSoftBodyHelpers_Draw,
	HingeConstraint_getLimitBiasFactor = CLIB.btHingeConstraint_getLimitBiasFactor,
	SoftBody_Anchor_getInfluence = CLIB.btSoftBody_Anchor_getInfluence,
	WorldImporter_createGeneric6DofConstraint = CLIB.btWorldImporter_createGeneric6DofConstraint,
	WorldImporter_createCylinderShapeY = CLIB.btWorldImporter_createCylinderShapeY,
	WorldImporter_createCylinderShapeX = CLIB.btWorldImporter_createCylinderShapeX,
	DbvtAabbMm_FromCE = CLIB.btDbvtAabbMm_FromCE,
	TypedConstraint_isEnabled = CLIB.btTypedConstraint_isEnabled,
	ConvexSeparatingDistanceUtil_getConservativeSeparatingDistance = CLIB.btConvexSeparatingDistanceUtil_getConservativeSeparatingDistance,
	DbvtBroadphase_setUpdates_done = CLIB.btDbvtBroadphase_setUpdates_done,
	CollisionShape_isConvex = CLIB.btCollisionShape_isConvex,
	GhostObject_convexSweepTest = CLIB.btGhostObject_convexSweepTest,
	SoftBody_appendAnchor5 = CLIB.btSoftBody_appendAnchor5,
	WorldImporter_createConvexTriangleMeshShape = CLIB.btWorldImporter_createConvexTriangleMeshShape,
	Dbvt_getLeaves = CLIB.btDbvt_getLeaves,
	MultibodyLink_getJointType = CLIB.btMultibodyLink_getJointType,
	TriangleShapeEx_applyTransform = CLIB.btTriangleShapeEx_applyTransform,
	WorldImporter_createConeTwistConstraint2 = CLIB.btWorldImporter_createConeTwistConstraint2,
	Generic6DofConstraint_setAngularLowerLimit = CLIB.btGeneric6DofConstraint_setAngularLowerLimit,
	DbvtBroadphase_getStageCurrent = CLIB.btDbvtBroadphase_getStageCurrent,
	TranslationalLimitMotor2_getServoMotor = CLIB.btTranslationalLimitMotor2_getServoMotor,
	ManifoldResult_setPersistentManifold = CLIB.btManifoldResult_setPersistentManifold,
	DynamicsWorld_getNumConstraints = CLIB.btDynamicsWorld_getNumConstraints,
	CollisionWorld_LocalShapeInfo_new = CLIB.btCollisionWorld_LocalShapeInfo_new,
	Generic6DofSpring2Constraint_setAngularLowerLimitReversed = CLIB.btGeneric6DofSpring2Constraint_setAngularLowerLimitReversed,
	ConvexPlaneCollisionAlgorithm_CreateFunc_setNumPerturbationIterations = CLIB.btConvexPlaneCollisionAlgorithm_CreateFunc_setNumPerturbationIterations,
	BulletWorldImporter_loadFile = CLIB.btBulletWorldImporter_loadFile,
	WorldImporter_createCompoundShape = CLIB.btWorldImporter_createCompoundShape,
	CollisionWorld_RayResultCallback_hasHit = CLIB.btCollisionWorld_RayResultCallback_hasHit,
	ManifoldPoint_setUserPersistentData = CLIB.btManifoldPoint_setUserPersistentData,
	ConvexPointCloudShape_getNumPoints = CLIB.btConvexPointCloudShape_getNumPoints,
	WorldImporter_createBvhTriangleMeshShape = CLIB.btWorldImporter_createBvhTriangleMeshShape,
	MultibodyLink_setAppliedConstraintTorque = CLIB.btMultibodyLink_setAppliedConstraintTorque,
	WorldImporter_createBoxShape = CLIB.btWorldImporter_createBoxShape,
	WheelInfo_delete = CLIB.btWheelInfo_delete,
	WheelInfo_updateWheel = CLIB.btWheelInfo_updateWheel,
	SoftBody_solveClusters = CLIB.btSoftBody_solveClusters,
	MultiBodySolverConstraint_setLinkA = CLIB.btMultiBodySolverConstraint_setLinkA,
	WheelInfo_setWheelsSuspensionForce = CLIB.btWheelInfo_setWheelsSuspensionForce,
	WheelInfo_setWheelsRadius = CLIB.btWheelInfo_setWheelsRadius,
	WheelInfo_setWheelsDampingRelaxation = CLIB.btWheelInfo_setWheelsDampingRelaxation,
	WheelInfo_setWheelsDampingCompression = CLIB.btWheelInfo_setWheelsDampingCompression,
	WheelInfo_setWheelDirectionCS = CLIB.btWheelInfo_setWheelDirectionCS,
	MultiBodySolverConstraint_setRelpos1CrossNormal = CLIB.btMultiBodySolverConstraint_setRelpos1CrossNormal,
	DbvtBroadphase_setUpdates_call = CLIB.btDbvtBroadphase_setUpdates_call,
	WheelInfo_setSuspensionStiffness = CLIB.btWheelInfo_setSuspensionStiffness,
	WheelInfo_setSuspensionRestLength1 = CLIB.btWheelInfo_setSuspensionRestLength1,
	SoftBody_getClusters = CLIB.btSoftBody_getClusters,
	UnionFind_unite = CLIB.btUnionFind_unite,
	WheelInfo_setSuspensionRelativeVelocity = CLIB.btWheelInfo_setSuspensionRelativeVelocity,
	ConeTwistConstraint_setMotorTargetInConstraintSpace = CLIB.btConeTwistConstraint_setMotorTargetInConstraintSpace,
	WheelInfo_setSteering = CLIB.btWheelInfo_setSteering,
	RigidBody_getVelocityInLocalPoint = CLIB.btRigidBody_getVelocityInLocalPoint,
	TypedConstraint_getUserConstraintPtr = CLIB.btTypedConstraint_getUserConstraintPtr,
	ActionInterfaceWrapper_new = CLIB.btActionInterfaceWrapper_new,
	TypedConstraint_setEnabled = CLIB.btTypedConstraint_setEnabled,
	AlignedObjectArray_btSoftBody_Tetra_push_back = CLIB.btAlignedObjectArray_btSoftBody_Tetra_push_back,
	RigidBody_btRigidBodyConstructionInfo_setLocalInertia = CLIB.btRigidBody_btRigidBodyConstructionInfo_setLocalInertia,
	AlignedObjectArray_btSoftBody_ClusterPtr_size = CLIB.btAlignedObjectArray_btSoftBody_ClusterPtr_size,
	WheelInfo_setMaxSuspensionTravelCm = CLIB.btWheelInfo_setMaxSuspensionTravelCm,
	ConeTwistConstraint_getSolveTwistLimit = CLIB.btConeTwistConstraint_getSolveTwistLimit,
	QuantizedBvh_calculateSerializeBufferSizeNew = CLIB.btQuantizedBvh_calculateSerializeBufferSizeNew,
	GImpactCompoundShape_addChildShape2 = CLIB.btGImpactCompoundShape_addChildShape2,
	WheelInfo_setMaxSuspensionForce = CLIB.btWheelInfo_setMaxSuspensionForce,
	WheelInfo_setFrictionSlip = CLIB.btWheelInfo_setFrictionSlip,
	RigidBody_integrateVelocities = CLIB.btRigidBody_integrateVelocities,
	WheelInfo_setDeltaRotation = CLIB.btWheelInfo_setDeltaRotation,
	SoftBody_Node_getLeaf = CLIB.btSoftBody_Node_getLeaf,
	Dbvt_sStkNPS_setMask = CLIB.btDbvt_sStkNPS_setMask,
	WheelInfo_setClippedInvContactDotSuspension = CLIB.btWheelInfo_setClippedInvContactDotSuspension,
	ConvexCast_CastResult_drawCoordSystem = CLIB.btConvexCast_CastResult_drawCoordSystem,
	WheelInfo_setClientInfo = CLIB.btWheelInfo_setClientInfo,
	WheelInfo_setChassisConnectionPointCS = CLIB.btWheelInfo_setChassisConnectionPointCS,
	WheelInfo_setBrake = CLIB.btWheelInfo_setBrake,
	ConvexTriangleMeshShape_new = CLIB.btConvexTriangleMeshShape_new,
	TranslationalLimitMotor_getLowerLimit = CLIB.btTranslationalLimitMotor_getLowerLimit,
	WheelInfo_getWorldTransform = CLIB.btWheelInfo_getWorldTransform,
	AlignedObjectArray_btCollisionObjectPtr_size = CLIB.btAlignedObjectArray_btCollisionObjectPtr_size,
	WheelInfo_getWheelsRadius = CLIB.btWheelInfo_getWheelsRadius,
	AngularLimit_getError = CLIB.btAngularLimit_getError,
	DiscreteCollisionDetectorInterface_Result_setShapeIdentifiersB = CLIB.btDiscreteCollisionDetectorInterface_Result_setShapeIdentifiersB,
	GImpactMeshShapePart_getVertexCount = CLIB.btGImpactMeshShapePart_getVertexCount,
	ConvexHullShape_new3 = CLIB.btConvexHullShape_new3,
	SoftBodyConcaveCollisionAlgorithm_clearCache = CLIB.btSoftBodyConcaveCollisionAlgorithm_clearCache,
	SoftBody_Cluster_setIdmass = CLIB.btSoftBody_Cluster_setIdmass,
	MultiBody_setupPrismatic = CLIB.btMultiBody_setupPrismatic,
	SoftBody_getMass = CLIB.btSoftBody_getMass,
	TriangleMesh_addIndex = CLIB.btTriangleMesh_addIndex,
	SoftBody_solveConstraints = CLIB.btSoftBody_solveConstraints,
	HingeAccumulatedAngleConstraint_new = CLIB.btHingeAccumulatedAngleConstraint_new,
	ActionInterface_delete = CLIB.btActionInterface_delete,
	GImpactMeshShapePart_TrimeshPrimitiveManager_new3 = CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_new3,
	BvhTriangleMeshShape_performRaycast = CLIB.btBvhTriangleMeshShape_performRaycast,
	SoftBody_clusterAImpulse = CLIB.btSoftBody_clusterAImpulse,
	CollisionWorld_ContactResultCallback_setCollisionFilterGroup = CLIB.btCollisionWorld_ContactResultCallback_setCollisionFilterGroup,
	CollisionShape_getAabb = CLIB.btCollisionShape_getAabb,
	SoftBody_appendFace2 = CLIB.btSoftBody_appendFace2,
	BroadphaseRayCallback_setLambda_max = CLIB.btBroadphaseRayCallback_setLambda_max,
	WheelInfo_getSuspensionRelativeVelocity = CLIB.btWheelInfo_getSuspensionRelativeVelocity,
	WheelInfo_getRollInfluence = CLIB.btWheelInfo_getRollInfluence,
	WheelInfo_getSkidInfo = CLIB.btWheelInfo_getSkidInfo,
	WheelInfo_getRotation = CLIB.btWheelInfo_getRotation,
	TriangleBuffer_new = CLIB.btTriangleBuffer_new,
	WheelInfo_getSteering = CLIB.btWheelInfo_getSteering,
	SliderConstraint_getCalculatedTransformA = CLIB.btSliderConstraint_getCalculatedTransformA,
	CollisionObject_getCompanionId = CLIB.btCollisionObject_getCompanionId,
	WheelInfo_getRaycastInfo = CLIB.btWheelInfo_getRaycastInfo,
	WheelInfo_getMaxSuspensionTravelCm = CLIB.btWheelInfo_getMaxSuspensionTravelCm,
	SimulationIslandManager_getUnionFind = CLIB.btSimulationIslandManager_getUnionFind,
	WheelInfo_setWorldTransform = CLIB.btWheelInfo_setWorldTransform,
	WheelInfo_getFrictionSlip = CLIB.btWheelInfo_getFrictionSlip,
	SliderConstraint_setDampingDirLin = CLIB.btSliderConstraint_setDampingDirLin,
	GeometryUtil_getPlaneEquationsFromVertices = CLIB.btGeometryUtil_getPlaneEquationsFromVertices,
	CollisionObject_serialize = CLIB.btCollisionObject_serialize,
	AngularLimit_new = CLIB.btAngularLimit_new,
	ConeShape_setHeight = CLIB.btConeShape_setHeight,
	BoxBoxDetector_getBox2 = CLIB.btBoxBoxDetector_getBox2,
	TriangleInfoMap_getEdgeDistanceThreshold = CLIB.btTriangleInfoMap_getEdgeDistanceThreshold,
	WheelInfo_getChassisConnectionPointCS = CLIB.btWheelInfo_getChassisConnectionPointCS,
	DefaultMotionState_new2 = CLIB.btDefaultMotionState_new2,
	DbvtNode_delete = CLIB.btDbvtNode_delete,
	ManifoldPoint_getNormalWorldOnB = CLIB.btManifoldPoint_getNormalWorldOnB,
	MultiBody_setCompanionId = CLIB.btMultiBody_setCompanionId,
	ContactSolverInfoData_setLinearSlop = CLIB.btContactSolverInfoData_setLinearSlop,
	WheelInfo_getBIsFrontWheel = CLIB.btWheelInfo_getBIsFrontWheel,
	SoftBody_Anchor_setC1 = CLIB.btSoftBody_Anchor_setC1,
	MultiBody_getLinearDamping = CLIB.btMultiBody_getLinearDamping,
	PersistentManifold_getContactBreakingThreshold = CLIB.btPersistentManifold_getContactBreakingThreshold,
	WheelInfo_RaycastInfo_delete = CLIB.btWheelInfo_RaycastInfo_delete,
	CollisionObject_setDeactivationTime = CLIB.btCollisionObject_setDeactivationTime,
	WheelInfo_RaycastInfo_setWheelDirectionWS = CLIB.btWheelInfo_RaycastInfo_setWheelDirectionWS,
	CollisionObject_getInterpolationWorldTransform = CLIB.btCollisionObject_getInterpolationWorldTransform,
	WheelInfo_RaycastInfo_setWheelAxleWS = CLIB.btWheelInfo_RaycastInfo_setWheelAxleWS,
	BulletWorldImporter_loadFile2 = CLIB.btBulletWorldImporter_loadFile2,
	CollisionShape_calculateTemporalAabb = CLIB.btCollisionShape_calculateTemporalAabb,
	RigidBody_setAngularFactor2 = CLIB.btRigidBody_setAngularFactor2,
	WheelInfo_RaycastInfo_setSuspensionLength = CLIB.btWheelInfo_RaycastInfo_setSuspensionLength,
	WheelInfo_RaycastInfo_setIsInContact = CLIB.btWheelInfo_RaycastInfo_setIsInContact,
	MinkowskiSumShape_getShapeA = CLIB.btMinkowskiSumShape_getShapeA,
	WheelInfo_RaycastInfo_setHardPointWS = CLIB.btWheelInfo_RaycastInfo_setHardPointWS,
	TriangleIndexVertexMaterialArray_new2 = CLIB.btTriangleIndexVertexMaterialArray_new2,
	SoftBody_appendNote7 = CLIB.btSoftBody_appendNote7,
	WheelInfo_RaycastInfo_setGroundObject = CLIB.btWheelInfo_RaycastInfo_setGroundObject,
	WheelInfo_RaycastInfo_setContactPointWS = CLIB.btWheelInfo_RaycastInfo_setContactPointWS,
	RotationalLimitMotor2_testLimitValue = CLIB.btRotationalLimitMotor2_testLimitValue,
	WheelInfo_RaycastInfo_setContactNormalWS = CLIB.btWheelInfo_RaycastInfo_setContactNormalWS,
	SliderConstraint_getFrameOffsetA = CLIB.btSliderConstraint_getFrameOffsetA,
	WheelInfo_RaycastInfo_getWheelDirectionWS = CLIB.btWheelInfo_RaycastInfo_getWheelDirectionWS,
	DbvtBroadphase_setAabbForceUpdate = CLIB.btDbvtBroadphase_setAabbForceUpdate,
	ConvexPlaneCollisionAlgorithm_CreateFunc_getMinimumPointsPerturbationThreshold = CLIB.btConvexPlaneCollisionAlgorithm_CreateFunc_getMinimumPointsPerturbationThreshold,
	MultiBodySolverConstraint_setSolverBodyIdA = CLIB.btMultiBodySolverConstraint_setSolverBodyIdA,
	WheelInfo_RaycastInfo_getSuspensionLength = CLIB.btWheelInfo_RaycastInfo_getSuspensionLength,
	WheelInfo_RaycastInfo_getIsInContact = CLIB.btWheelInfo_RaycastInfo_getIsInContact,
	WheelInfo_RaycastInfo_getHardPointWS = CLIB.btWheelInfo_RaycastInfo_getHardPointWS,
	Dbvt_update5 = CLIB.btDbvt_update5,
	CompoundShapeChild_array_at = CLIB.btCompoundShapeChild_array_at,
	SoftBody_appendLinearJoint2 = CLIB.btSoftBody_appendLinearJoint2,
	WheelInfo_RaycastInfo_getContactNormalWS = CLIB.btWheelInfo_RaycastInfo_getContactNormalWS,
	Chunk_setDna_nr = CLIB.btChunk_setDna_nr,
	TranslationalLimitMotor_testLimitValue = CLIB.btTranslationalLimitMotor_testLimitValue,
	BoxShape_new = CLIB.btBoxShape_new,
	GhostObject_getOverlappingObject = CLIB.btGhostObject_getOverlappingObject,
	MultiBody_getMaxAppliedImpulse = CLIB.btMultiBody_getMaxAppliedImpulse,
	WheelInfoConstructionInfo_setWheelRadius = CLIB.btWheelInfoConstructionInfo_setWheelRadius,
	ContactSolverInfoData_new = CLIB.btContactSolverInfoData_new,
	WheelInfoConstructionInfo_setWheelDirectionCS = CLIB.btWheelInfoConstructionInfo_setWheelDirectionCS,
	WheelInfoConstructionInfo_setWheelAxleCS = CLIB.btWheelInfoConstructionInfo_setWheelAxleCS,
	MultiBodySolverConstraint_getAngularComponentA = CLIB.btMultiBodySolverConstraint_getAngularComponentA,
	HeightfieldTerrainShape_new2 = CLIB.btHeightfieldTerrainShape_new2,
	AlignedObjectArray_btIndexedMesh_resizeNoInitialize = CLIB.btAlignedObjectArray_btIndexedMesh_resizeNoInitialize,
	WheelInfoConstructionInfo_setSuspensionStiffness = CLIB.btWheelInfoConstructionInfo_setSuspensionStiffness,
	MultiBody_applyDeltaVeeMultiDof = CLIB.btMultiBody_applyDeltaVeeMultiDof,
	WheelInfoConstructionInfo_setSuspensionRestLength = CLIB.btWheelInfoConstructionInfo_setSuspensionRestLength,
	CompoundCollisionAlgorithm_new = CLIB.btCompoundCollisionAlgorithm_new,
	CollisionObjectWrapper_getParent = CLIB.btCollisionObjectWrapper_getParent,
	GImpactBvh_getGlobalBox = CLIB.btGImpactBvh_getGlobalBox,
	WheelInfoConstructionInfo_setMaxSuspensionTravelCm = CLIB.btWheelInfoConstructionInfo_setMaxSuspensionTravelCm,
	DbvtBroadphase_getSets = CLIB.btDbvtBroadphase_getSets,
	WheelInfoConstructionInfo_setMaxSuspensionForce = CLIB.btWheelInfoConstructionInfo_setMaxSuspensionForce,
	WheelInfoConstructionInfo_setFrictionSlip = CLIB.btWheelInfoConstructionInfo_setFrictionSlip,
	RigidBody_setMassProps = CLIB.btRigidBody_setMassProps,
	WheelInfoConstructionInfo_setChassisConnectionCS = CLIB.btWheelInfoConstructionInfo_setChassisConnectionCS,
	WheelInfoConstructionInfo_setBIsFrontWheel = CLIB.btWheelInfoConstructionInfo_setBIsFrontWheel,
	ManifoldPoint_setCombinedFriction = CLIB.btManifoldPoint_setCombinedFriction,
	WheelInfoConstructionInfo_getWheelsDampingRelaxation = CLIB.btWheelInfoConstructionInfo_getWheelsDampingRelaxation,
	MultiBody_stepPositionsMultiDof2 = CLIB.btMultiBody_stepPositionsMultiDof2,
	ConvexPolyhedron_setMC = CLIB.btConvexPolyhedron_setMC,
	WheelInfoConstructionInfo_getWheelRadius = CLIB.btWheelInfoConstructionInfo_getWheelRadius,
	PolyhedralConvexShape_getConvexPolyhedron = CLIB.btPolyhedralConvexShape_getConvexPolyhedron,
	SoftBody_appendAngularJoint = CLIB.btSoftBody_appendAngularJoint,
	TypedConstraint_getUserConstraintId = CLIB.btTypedConstraint_getUserConstraintId,
	MotionState_setWorldTransform = CLIB.btMotionState_setWorldTransform,
	SoftBody_appendLink8 = CLIB.btSoftBody_appendLink8,
	WheelInfoConstructionInfo_getWheelAxleCS = CLIB.btWheelInfoConstructionInfo_getWheelAxleCS,
	WheelInfoConstructionInfo_getSuspensionStiffness = CLIB.btWheelInfoConstructionInfo_getSuspensionStiffness,
	WheelInfoConstructionInfo_getSuspensionRestLength = CLIB.btWheelInfoConstructionInfo_getSuspensionRestLength,
	ContactSolverInfoData_setDamping = CLIB.btContactSolverInfoData_setDamping,
	HingeConstraint_setLimit3 = CLIB.btHingeConstraint_setLimit3,
	QuantizedBvh_getAlignmentSerializationPadding = CLIB.btQuantizedBvh_getAlignmentSerializationPadding,
	WheelInfoConstructionInfo_getMaxSuspensionForce = CLIB.btWheelInfoConstructionInfo_getMaxSuspensionForce,
	WheelInfoConstructionInfo_getFrictionSlip = CLIB.btWheelInfoConstructionInfo_getFrictionSlip,
	SoftBody_Material_getKAST = CLIB.btSoftBody_Material_getKAST,
	WheelInfoConstructionInfo_getChassisConnectionCS = CLIB.btWheelInfoConstructionInfo_getChassisConnectionCS,
	Triangle_getVertex2 = CLIB.btTriangle_getVertex2,
	WheelInfoConstructionInfo_getBIsFrontWheel = CLIB.btWheelInfoConstructionInfo_getBIsFrontWheel,
	GImpactCompoundShape_getCompoundPrimitiveManager = CLIB.btGImpactCompoundShape_getCompoundPrimitiveManager,
	GImpactCompoundShape_CompoundPrimitiveManager_new3 = CLIB.btGImpactCompoundShape_CompoundPrimitiveManager_new3,
	PrimitiveManagerBase_get_primitive_box = CLIB.btPrimitiveManagerBase_get_primitive_box,
	TypedConstraint_btConstraintInfo1_getNumConstraintRows = CLIB.btTypedConstraint_btConstraintInfo1_getNumConstraintRows,
	GImpactBvh_getNodeTriangle = CLIB.btGImpactBvh_getNodeTriangle,
	RigidBody_applyTorqueImpulse = CLIB.btRigidBody_applyTorqueImpulse,
	VoronoiSimplexSolver_updateClosestVectorAndPoints = CLIB.btVoronoiSimplexSolver_updateClosestVectorAndPoints,
	PolyhedralConvexShape_getNumPlanes = CLIB.btPolyhedralConvexShape_getNumPlanes,
	VoronoiSimplexSolver_setNumVertices = CLIB.btVoronoiSimplexSolver_setNumVertices,
	ConvexHullShape_addPoint2 = CLIB.btConvexHullShape_addPoint2,
	CollisionObject_getWorldTransform = CLIB.btCollisionObject_getWorldTransform,
	VoronoiSimplexSolver_setNeedsUpdate = CLIB.btVoronoiSimplexSolver_setNeedsUpdate,
	CylinderShapeX_new = CLIB.btCylinderShapeX_new,
	DbvtAabbMm_Lengths = CLIB.btDbvtAabbMm_Lengths,
	HingeConstraint_setMotorTarget2 = CLIB.btHingeConstraint_setMotorTarget2,
	MultiBodyDynamicsWorld_clearMultiBodyForces = CLIB.btMultiBodyDynamicsWorld_clearMultiBodyForces,
	CollisionObject_setInterpolationLinearVelocity = CLIB.btCollisionObject_setInterpolationLinearVelocity,
	AABB_getMax = CLIB.btAABB_getMax,
	ManifoldPoint_getIndex0 = CLIB.btManifoldPoint_getIndex0,
	MLCPSolverInterface_delete = CLIB.btMLCPSolverInterface_delete,
	VoronoiSimplexSolver_setLastW = CLIB.btVoronoiSimplexSolver_setLastW,
	MultibodyLink_setEVector = CLIB.btMultibodyLink_setEVector,
	RigidBody_setAngularFactor = CLIB.btRigidBody_setAngularFactor,
	SoftBody_Config_setKVCF = CLIB.btSoftBody_Config_setKVCF,
	Dbvt_IWriter_WriteLeaf = CLIB.btDbvt_IWriter_WriteLeaf,
	VoronoiSimplexSolver_setCachedV = CLIB.btVoronoiSimplexSolver_setCachedV,
	TranslationalLimitMotor2_delete = CLIB.btTranslationalLimitMotor2_delete,
	VoronoiSimplexSolver_setCachedP2 = CLIB.btVoronoiSimplexSolver_setCachedP2,
	MultibodyLink_setDofCount = CLIB.btMultibodyLink_setDofCount,
	AABB_delete = CLIB.btAABB_delete,
	VoronoiSimplexSolver_setCachedP1 = CLIB.btVoronoiSimplexSolver_setCachedP1,
	VoronoiSimplexSolver_setCachedBC = CLIB.btVoronoiSimplexSolver_setCachedBC,
	MultiBody_stepPositionsMultiDof = CLIB.btMultiBody_stepPositionsMultiDof,
	SoftBody_AJoint_IControl_new = CLIB.btSoftBody_AJoint_IControl_new,
	VoronoiSimplexSolver_reset = CLIB.btVoronoiSimplexSolver_reset,
	VoronoiSimplexSolver_removeVertex = CLIB.btVoronoiSimplexSolver_removeVertex,
	VoronoiSimplexSolver_reduceVertices = CLIB.btVoronoiSimplexSolver_reduceVertices,
	SoftBody_ImplicitFnWrapper_new = CLIB.btSoftBody_ImplicitFnWrapper_new,
	ScaledBvhTriangleMeshShape_new = CLIB.btScaledBvhTriangleMeshShape_new,
	BvhTree_delete = CLIB.btBvhTree_delete,
	RotationalLimitMotor2_getCurrentLimitErrorHi = CLIB.btRotationalLimitMotor2_getCurrentLimitErrorHi,
	VoronoiSimplexSolver_pointOutsideOfPlane = CLIB.btVoronoiSimplexSolver_pointOutsideOfPlane,
	VoronoiSimplexSolver_numVertices = CLIB.btVoronoiSimplexSolver_numVertices,
	VoronoiSimplexSolver_maxVertex = CLIB.btVoronoiSimplexSolver_maxVertex,
	VoronoiSimplexSolver_inSimplex = CLIB.btVoronoiSimplexSolver_inSimplex,
	RaycastVehicle_btVehicleTuning_setMaxSuspensionTravelCm = CLIB.btRaycastVehicle_btVehicleTuning_setMaxSuspensionTravelCm,
	AABB_setMax = CLIB.btAABB_setMax,
	VoronoiSimplexSolver_getSimplexPointsQ = CLIB.btVoronoiSimplexSolver_getSimplexPointsQ,
	VoronoiSimplexSolver_getSimplexPointsP = CLIB.btVoronoiSimplexSolver_getSimplexPointsP,
	GjkPairDetector_new2 = CLIB.btGjkPairDetector_new2,
	CollisionWorld_AllHitsRayResultCallback_getRayFromWorld = CLIB.btCollisionWorld_AllHitsRayResultCallback_getRayFromWorld,
	SoftBody_cutLink = CLIB.btSoftBody_cutLink,
	VoronoiSimplexSolver_getNeedsUpdate = CLIB.btVoronoiSimplexSolver_getNeedsUpdate,
	TriangleMesh_addTriangle2 = CLIB.btTriangleMesh_addTriangle2,
	SoftBody_setSoftBodySolver = CLIB.btSoftBody_setSoftBodySolver,
	ContactSolverInfoData_setWarmstartingFactor = CLIB.btContactSolverInfoData_setWarmstartingFactor,
	AngularLimit_test = CLIB.btAngularLimit_test,
	SoftBody_defaultCollisionHandler2 = CLIB.btSoftBody_defaultCollisionHandler2,
	TranslationalLimitMotor2_getTargetVelocity = CLIB.btTranslationalLimitMotor2_getTargetVelocity,
	DefaultCollisionConfiguration_setPlaneConvexMultipointIterations3 = CLIB.btDefaultCollisionConfiguration_setPlaneConvexMultipointIterations3,
	Point2PointConstraint_getSetting = CLIB.btPoint2PointConstraint_getSetting,
	SoftBody_Config_setKCHR = CLIB.btSoftBody_Config_setKCHR,
	VoronoiSimplexSolver_getCachedValidClosest = CLIB.btVoronoiSimplexSolver_getCachedValidClosest,
	VoronoiSimplexSolver_getCachedV = CLIB.btVoronoiSimplexSolver_getCachedV,
	Chunk_delete = CLIB.btChunk_delete,
	MultiBody_setBaseWorldTransform = CLIB.btMultiBody_setBaseWorldTransform,
	ConvexShape_batchedUnitVectorGetSupportingVertexWithoutMargin = CLIB.btConvexShape_batchedUnitVectorGetSupportingVertexWithoutMargin,
	TypedConstraint_btConstraintInfo2_getLowerLimit = CLIB.btTypedConstraint_btConstraintInfo2_getLowerLimit,
	Hinge2Constraint_getAxis1 = CLIB.btHinge2Constraint_getAxis1,
	BroadphasePair_getAlgorithm = CLIB.btBroadphasePair_getAlgorithm,
	CollisionWorld_setForceUpdateAllAabbs = CLIB.btCollisionWorld_setForceUpdateAllAabbs,
	IndexedMesh_delete = CLIB.btIndexedMesh_delete,
	CollisionObject_getCollisionShape = CLIB.btCollisionObject_getCollisionShape,
	BroadphaseInterface_aabbTest = CLIB.btBroadphaseInterface_aabbTest,
	MultiBodySolverConstraint_setJacAindex = CLIB.btMultiBodySolverConstraint_setJacAindex,
	AlignedObjectArray_btPersistentManifoldPtr_delete = CLIB.btAlignedObjectArray_btPersistentManifoldPtr_delete,
	MultimaterialTriangleMeshShape_new3 = CLIB.btMultimaterialTriangleMeshShape_new3,
	VoronoiSimplexSolver_fullSimplex = CLIB.btVoronoiSimplexSolver_fullSimplex,
	VoronoiSimplexSolver_emptySimplex = CLIB.btVoronoiSimplexSolver_emptySimplex,
	VoronoiSimplexSolver_compute_points = CLIB.btVoronoiSimplexSolver_compute_points,
	SoftBody_updateClusters = CLIB.btSoftBody_updateClusters,
	VoronoiSimplexSolver_closestPtPointTetrahedron = CLIB.btVoronoiSimplexSolver_closestPtPointTetrahedron,
	DefaultCollisionConstructionInfo_setDefaultMaxCollisionAlgorithmPoolSize = CLIB.btDefaultCollisionConstructionInfo_setDefaultMaxCollisionAlgorithmPoolSize,
	UniversalConstraint_getAnchor2 = CLIB.btUniversalConstraint_getAnchor2,
	MultiBody_clearConstraintForces = CLIB.btMultiBody_clearConstraintForces,
	MultiBody_setJointPos = CLIB.btMultiBody_setJointPos,
	DefaultMotionState_getCenterOfMassOffset = CLIB.btDefaultMotionState_getCenterOfMassOffset,
	ConeTwistConstraint_isPastSwingLimit = CLIB.btConeTwistConstraint_isPastSwingLimit,
	VoronoiSimplexSolver_backup_closest = CLIB.btVoronoiSimplexSolver_backup_closest,
	VoronoiSimplexSolver_addVertex = CLIB.btVoronoiSimplexSolver_addVertex,
	DynamicsWorld_setWorldUserInfo = CLIB.btDynamicsWorld_setWorldUserInfo,
	VoronoiSimplexSolver_new = CLIB.btVoronoiSimplexSolver_new,
	GImpactBvh_getLeftNode = CLIB.btGImpactBvh_getLeftNode,
	DispatcherInfo_getUseEpa = CLIB.btDispatcherInfo_getUseEpa,
	CharacterControllerInterface_preStep = CLIB.btCharacterControllerInterface_preStep,
	AlignedObjectArray_btSoftBody_Anchor_resizeNoInitialize = CLIB.btAlignedObjectArray_btSoftBody_Anchor_resizeNoInitialize,
	SliderConstraint_getUpperAngLimit = CLIB.btSliderConstraint_getUpperAngLimit,
	SubSimplexClosestResult_setUsedVertices = CLIB.btSubSimplexClosestResult_setUsedVertices,
	ManifoldPoint_setLateralFrictionDir2 = CLIB.btManifoldPoint_setLateralFrictionDir2,
	SubSimplexClosestResult_setDegenerate = CLIB.btSubSimplexClosestResult_setDegenerate,
	TranslationalLimitMotor2_getSpringDamping = CLIB.btTranslationalLimitMotor2_getSpringDamping,
	CompoundShape_getChildTransform = CLIB.btCompoundShape_getChildTransform,
	SubSimplexClosestResult_setClosestPointOnSimplex = CLIB.btSubSimplexClosestResult_setClosestPointOnSimplex,
	SubSimplexClosestResult_setBarycentricCoordinates5 = CLIB.btSubSimplexClosestResult_setBarycentricCoordinates5,
	SoftBody_Pose_getScl = CLIB.btSoftBody_Pose_getScl,
	MultiBodySolverConstraint_getContactNormal2 = CLIB.btMultiBodySolverConstraint_getContactNormal2,
	SubSimplexClosestResult_setBarycentricCoordinates2 = CLIB.btSubSimplexClosestResult_setBarycentricCoordinates2,
	TranslationalLimitMotor2_setServoTarget = CLIB.btTranslationalLimitMotor2_setServoTarget,
	SoftBody_Anchor_getLocal = CLIB.btSoftBody_Anchor_getLocal,
	SubSimplexClosestResult_setBarycentricCoordinates = CLIB.btSubSimplexClosestResult_setBarycentricCoordinates,
	ContactSolverInfoData_setMinimumSolverBatchSize = CLIB.btContactSolverInfoData_setMinimumSolverBatchSize,
	PolyhedralConvexShape_getNumEdges = CLIB.btPolyhedralConvexShape_getNumEdges,
	SubSimplexClosestResult_getUsedVertices = CLIB.btSubSimplexClosestResult_getUsedVertices,
	DantzigSolver_new = CLIB.btDantzigSolver_new,
	SubSimplexClosestResult_getClosestPointOnSimplex = CLIB.btSubSimplexClosestResult_getClosestPointOnSimplex,
	SubSimplexClosestResult_getBarycentricCoords = CLIB.btSubSimplexClosestResult_getBarycentricCoords,
	Box2dShape_new = CLIB.btBox2dShape_new,
	UsageBitfield_setUsedVertexD = CLIB.btUsageBitfield_setUsedVertexD,
	MultiBodyConstraint_internalSetAppliedImpulse = CLIB.btMultiBodyConstraint_internalSetAppliedImpulse,
	PrimitiveTriangle_new = CLIB.btPrimitiveTriangle_new,
	MultiBody_processDeltaVeeMultiDof2 = CLIB.btMultiBody_processDeltaVeeMultiDof2,
	DbvtBroadphase_setPid = CLIB.btDbvtBroadphase_setPid,
	UsageBitfield_setUsedVertexC = CLIB.btUsageBitfield_setUsedVertexC,
	CollisionWorld_RayResultCallback_getCollisionObject = CLIB.btCollisionWorld_RayResultCallback_getCollisionObject,
	UsageBitfield_setUsedVertexB = CLIB.btUsageBitfield_setUsedVertexB,
	MultiBodySolverConstraint_setRelpos2CrossNormal = CLIB.btMultiBodySolverConstraint_setRelpos2CrossNormal,
	UsageBitfield_setUnused4 = CLIB.btUsageBitfield_setUnused4,
	CollisionWorld_LocalConvexResult_getLocalShapeInfo = CLIB.btCollisionWorld_LocalConvexResult_getLocalShapeInfo,
	UsageBitfield_setUnused3 = CLIB.btUsageBitfield_setUnused3,
	UsageBitfield_setUnused2 = CLIB.btUsageBitfield_setUnused2,
	WorldImporter_createGeneric6DofSpringConstraint = CLIB.btWorldImporter_createGeneric6DofSpringConstraint,
	ConvexInternalShape_getImplicitShapeDimensions = CLIB.btConvexInternalShape_getImplicitShapeDimensions,
	BroadphaseProxy_isConcave = CLIB.btBroadphaseProxy_isConcave,
	UsageBitfield_getUsedVertexD = CLIB.btUsageBitfield_getUsedVertexD,
	UsageBitfield_getUsedVertexC = CLIB.btUsageBitfield_getUsedVertexC,
	UsageBitfield_getUsedVertexB = CLIB.btUsageBitfield_getUsedVertexB,
	Generic6DofSpring2Constraint_new = CLIB.btGeneric6DofSpring2Constraint_new,
	MultiBodyDynamicsWorld_removeMultiBodyConstraint = CLIB.btMultiBodyDynamicsWorld_removeMultiBodyConstraint,
	ConeTwistConstraint_getFlags = CLIB.btConeTwistConstraint_getFlags,
	ConvexCast_CastResult_delete = CLIB.btConvexCast_CastResult_delete,
	CollisionObject_isStaticObject = CLIB.btCollisionObject_isStaticObject,
	UsageBitfield_getUnused4 = CLIB.btUsageBitfield_getUnused4,
	UsageBitfield_getUnused2 = CLIB.btUsageBitfield_getUnused2,
	MultiBody_getBasePos = CLIB.btMultiBody_getBasePos,
	RaycastVehicle_btVehicleTuning_getSuspensionCompression = CLIB.btRaycastVehicle_btVehicleTuning_getSuspensionCompression,
	BroadphaseInterface_getBroadphaseAabb = CLIB.btBroadphaseInterface_getBroadphaseAabb,
	UsageBitfield_getUnused1 = CLIB.btUsageBitfield_getUnused1,
	AlignedObjectArray_btSoftBody_Tetra_at = CLIB.btAlignedObjectArray_btSoftBody_Tetra_at,
	VehicleRaycaster_delete = CLIB.btVehicleRaycaster_delete,
	ConvexTriangleCallback_setManifoldPtr = CLIB.btConvexTriangleCallback_setManifoldPtr,
	TranslationalLimitMotor2_new2 = CLIB.btTranslationalLimitMotor2_new2,
	VehicleRaycaster_castRay = CLIB.btVehicleRaycaster_castRay,
	ContactSolverInfoData_getTau = CLIB.btContactSolverInfoData_getTau,
	HingeConstraint_getHingeAngle = CLIB.btHingeConstraint_getHingeAngle,
	SoftBody_SContact_setWeights = CLIB.btSoftBody_SContact_setWeights,
	VehicleRaycaster_btVehicleRaycasterResult_setHitPointInWorld = CLIB.btVehicleRaycaster_btVehicleRaycasterResult_setHitPointInWorld,
	ConvexShape_localGetSupportVertexWithoutMarginNonVirtual = CLIB.btConvexShape_localGetSupportVertexWithoutMarginNonVirtual,
	VehicleRaycaster_btVehicleRaycasterResult_setHitNormalInWorld = CLIB.btVehicleRaycaster_btVehicleRaycasterResult_setHitNormalInWorld,
	SoftBody_Link_getC2 = CLIB.btSoftBody_Link_getC2,
	MultiBody_useGlobalVelocities = CLIB.btMultiBody_useGlobalVelocities,
	VehicleRaycaster_btVehicleRaycasterResult_setDistFraction = CLIB.btVehicleRaycaster_btVehicleRaycasterResult_setDistFraction,
	CollisionShape_isConcave = CLIB.btCollisionShape_isConcave,
	VehicleRaycaster_btVehicleRaycasterResult_getHitPointInWorld = CLIB.btVehicleRaycaster_btVehicleRaycasterResult_getHitPointInWorld,
	SoftBody_RayFromToCaster_getFace = CLIB.btSoftBody_RayFromToCaster_getFace,
	Convex2dConvex2dAlgorithm_CreateFunc_setNumPerturbationIterations = CLIB.btConvex2dConvex2dAlgorithm_CreateFunc_setNumPerturbationIterations,
	Dbvt_sStkNP_getNode = CLIB.btDbvt_sStkNP_getNode,
	VehicleRaycaster_btVehicleRaycasterResult_getDistFraction = CLIB.btVehicleRaycaster_btVehicleRaycasterResult_getDistFraction,
	VehicleRaycaster_btVehicleRaycasterResult_new = CLIB.btVehicleRaycaster_btVehicleRaycasterResult_new,
	MultiBody_getLink = CLIB.btMultiBody_getLink,
	PolyhedralConvexShape_getPlane = CLIB.btPolyhedralConvexShape_getPlane,
	UniversalConstraint_setUpperLimit = CLIB.btUniversalConstraint_setUpperLimit,
	BroadphaseRayCallbackWrapper_new = CLIB.btBroadphaseRayCallbackWrapper_new,
	UniversalConstraint_setLowerLimit = CLIB.btUniversalConstraint_setLowerLimit,
	GImpactBvh_setNodeBound = CLIB.btGImpactBvh_setNodeBound,
	Dispatcher_getInternalManifoldPointer = CLIB.btDispatcher_getInternalManifoldPointer,
	MultibodyLink_getParent = CLIB.btMultibodyLink_getParent,
	DefaultMotionState_new = CLIB.btDefaultMotionState_new,
	UniversalConstraint_getAxis2 = CLIB.btUniversalConstraint_getAxis2,
	DbvtProxy_setStage = CLIB.btDbvtProxy_setStage,
	RotationalLimitMotor2_getEnableMotor = CLIB.btRotationalLimitMotor2_getEnableMotor,
	SoftBodyConcaveCollisionAlgorithm_new = CLIB.btSoftBodyConcaveCollisionAlgorithm_new,
	HingeAccumulatedAngleConstraint_new2 = CLIB.btHingeAccumulatedAngleConstraint_new2,
	MultiBody_setupFixed = CLIB.btMultiBody_setupFixed,
	Generic6DofConstraint_getRelativePivotPosition = CLIB.btGeneric6DofConstraint_getRelativePivotPosition,
	UniversalConstraint_new = CLIB.btUniversalConstraint_new,
	UnionFind_sortIslands = CLIB.btUnionFind_sortIslands,
	UnionFind_reset = CLIB.btUnionFind_reset,
	MultiBodyConstraint_getIslandIdB = CLIB.btMultiBodyConstraint_getIslandIdB,
	WheelInfoConstructionInfo_getMaxSuspensionTravelCm = CLIB.btWheelInfoConstructionInfo_getMaxSuspensionTravelCm,
	DiscreteDynamicsWorld_setLatencyMotionStateInterpolation = CLIB.btDiscreteDynamicsWorld_setLatencyMotionStateInterpolation,
	UnionFind_getNumElements = CLIB.btUnionFind_getNumElements,
	WorldImporter_getNumRigidBodies = CLIB.btWorldImporter_getNumRigidBodies,
	UnionFind_find2 = CLIB.btUnionFind_find2,
	SoftBodyWorldInfo_getWater_density = CLIB.btSoftBodyWorldInfo_getWater_density,
	UnionFind_find = CLIB.btUnionFind_find,
	ManifoldResult_setBody1Wrap = CLIB.btManifoldResult_setBody1Wrap,
	SoftBodyConcaveCollisionAlgorithm_SwappedCreateFunc_new = CLIB.btSoftBodyConcaveCollisionAlgorithm_SwappedCreateFunc_new,
	AABB_plane_classify = CLIB.btAABB_plane_classify,
	Element_delete = CLIB.btElement_delete,
	SoftBody_appendNote6 = CLIB.btSoftBody_appendNote6,
	SoftBody_clusterCom = CLIB.btSoftBody_clusterCom,
	MultiBodyConstraint_setPosition = CLIB.btMultiBodyConstraint_setPosition,
	Element_setSz = CLIB.btElement_setSz,
	CollisionObject_setCollisionShape = CLIB.btCollisionObject_setCollisionShape,
	Element_setId = CLIB.btElement_setId,
	BulletWorldImporter_new2 = CLIB.btBulletWorldImporter_new2,
	Element_getSz = CLIB.btElement_getSz,
	Element_getId = CLIB.btElement_getId,
	Dbvt_update4 = CLIB.btDbvt_update4,
	Element_new = CLIB.btElement_new,
	BulletWorldImporter_loadFileFromMemory = CLIB.btBulletWorldImporter_loadFileFromMemory,
	Box2dShape_getNormals = CLIB.btBox2dShape_getNormals,
	CollisionObject_calculateSerializeBufferSize = CLIB.btCollisionObject_calculateSerializeBufferSize,
	Dbvt_nearest = CLIB.btDbvt_nearest,
	UniformScalingShape_getUniformScalingFactor = CLIB.btUniformScalingShape_getUniformScalingFactor,
	CollisionShape_isCompound = CLIB.btCollisionShape_isCompound,
	RigidBody_getInvInertiaTensorWorld = CLIB.btRigidBody_getInvInertiaTensorWorld,
	SoftBody_addVelocity = CLIB.btSoftBody_addVelocity,
	MultiBody_hasFixedBase = CLIB.btMultiBody_hasFixedBase,
	SoftBody_RayFromToCaster_rayFromToTriangle = CLIB.btSoftBody_RayFromToCaster_rayFromToTriangle,
	UniformScalingShape_getChildShape = CLIB.btUniformScalingShape_getChildShape,
	UniformScalingShape_new = CLIB.btUniformScalingShape_new,
	TriangleMeshShape_getLocalAabbMax = CLIB.btTriangleMeshShape_getLocalAabbMax,
	MultiBody_setMaxCoordinateVelocity = CLIB.btMultiBody_setMaxCoordinateVelocity,
	SoftBody_Link_setBbending = CLIB.btSoftBody_Link_setBbending,
	SphereShape_new = CLIB.btSphereShape_new,
	AlignedObjectArray_btSoftBody_Node_resizeNoInitialize = CLIB.btAlignedObjectArray_btSoftBody_Node_resizeNoInitialize,
	MultiBodyDynamicsWorld_addMultiBodyConstraint = CLIB.btMultiBodyDynamicsWorld_addMultiBodyConstraint,
	SphereTriangleCollisionAlgorithm_new = CLIB.btSphereTriangleCollisionAlgorithm_new,
	Chunk_getOldPtr = CLIB.btChunk_getOldPtr,
	QuantizedBvhTree_get_node_pointer = CLIB.btQuantizedBvhTree_get_node_pointer,
	SoftBodyWorldInfo_getWater_offset = CLIB.btSoftBodyWorldInfo_getWater_offset,
	QuantizedBvh_quantize = CLIB.btQuantizedBvh_quantize,
	CollisionObject_isStaticOrKinematicObject = CLIB.btCollisionObject_isStaticOrKinematicObject,
	SoftBody_AJoint_IControlWrapper_setWrapperData = CLIB.btSoftBody_AJoint_IControlWrapper_setWrapperData,
	SoftBody_Body_applyVImpulse = CLIB.btSoftBody_Body_applyVImpulse,
	DbvtBroadphase_benchmark = CLIB.btDbvtBroadphase_benchmark,
	GImpactShapeInterface_postUpdate = CLIB.btGImpactShapeInterface_postUpdate,
	StridingMeshInterface_calculateSerializeBufferSize = CLIB.btStridingMeshInterface_calculateSerializeBufferSize,
	Generic6DofSpring2Constraint_isLimited = CLIB.btGeneric6DofSpring2Constraint_isLimited,
	SoftBody_Config_getKLF = CLIB.btSoftBody_Config_getKLF,
	ContactSolverInfoData_setErp = CLIB.btContactSolverInfoData_setErp,
	AngularLimit_getCorrection = CLIB.btAngularLimit_getCorrection,
	MultiBody_getLinkTorque = CLIB.btMultiBody_getLinkTorque,
	AngularLimit_fit = CLIB.btAngularLimit_fit,
	CollisionWorld_LocalConvexResult_setHitPointLocal = CLIB.btCollisionWorld_LocalConvexResult_setHitPointLocal,
	TypedConstraint_setUserConstraintType = CLIB.btTypedConstraint_setUserConstraintType,
	TypedConstraint_setUserConstraintPtr = CLIB.btTypedConstraint_setUserConstraintPtr,
	ConeTwistConstraint_setLimit2 = CLIB.btConeTwistConstraint_setLimit2,
	DefaultSerializer_internalAlloc = CLIB.btDefaultSerializer_internalAlloc,
	ConvexCast_CastResult_getNormal = CLIB.btConvexCast_CastResult_getNormal,
	GImpactMeshShapePart_TrimeshPrimitiveManager_getMargin = CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_getMargin,
	TypedConstraint_setupSolverConstraint = CLIB.btTypedConstraint_setupSolverConstraint,
	TypedConstraint_setParam2 = CLIB.btTypedConstraint_setParam2,
	TypedConstraint_setParam = CLIB.btTypedConstraint_setParam,
	TypedConstraint_setOverrideNumSolverIterations = CLIB.btTypedConstraint_setOverrideNumSolverIterations,
	TypedConstraint_setJointFeedback = CLIB.btTypedConstraint_setJointFeedback,
	WheelInfo_setRotation = CLIB.btWheelInfo_setRotation,
	SliderConstraint_getSoftnessDirAng = CLIB.btSliderConstraint_getSoftnessDirAng,
	TypedConstraint_setDbgDrawSize = CLIB.btTypedConstraint_setDbgDrawSize,
	RigidBody_btRigidBodyConstructionInfo_new = CLIB.btRigidBody_btRigidBodyConstructionInfo_new,
	TypedConstraint_setBreakingImpulseThreshold = CLIB.btTypedConstraint_setBreakingImpulseThreshold,
	MultiBody_getCompanionId = CLIB.btMultiBody_getCompanionId,
	TypedConstraint_serialize = CLIB.btTypedConstraint_serialize,
	SoftBody_SolverState_getUpdmrg = CLIB.btSoftBody_SolverState_getUpdmrg,
	TypedConstraint_internalSetAppliedImpulse = CLIB.btTypedConstraint_internalSetAppliedImpulse,
	MultiBody_getBaseName = CLIB.btMultiBody_getBaseName,
	WheelInfoConstructionInfo_setWheelsDampingCompression = CLIB.btWheelInfoConstructionInfo_setWheelsDampingCompression,
	TypedConstraint_getUserConstraintType = CLIB.btTypedConstraint_getUserConstraintType,
	TypedConstraint_getRigidBodyB = CLIB.btTypedConstraint_getRigidBodyB,
	SliderConstraint_getMaxAngMotorForce = CLIB.btSliderConstraint_getMaxAngMotorForce,
	TranslationalLimitMotor_setStopCFM = CLIB.btTranslationalLimitMotor_setStopCFM,
	GjkPairDetector_getCatchDegeneracies = CLIB.btGjkPairDetector_getCatchDegeneracies,
	MultiBodyDynamicsWorld_addMultiBody2 = CLIB.btMultiBodyDynamicsWorld_addMultiBody2,
	TypedConstraint_getParam2 = CLIB.btTypedConstraint_getParam2,
	ManifoldPoint_setIndex1 = CLIB.btManifoldPoint_setIndex1,
	TypedConstraint_getParam = CLIB.btTypedConstraint_getParam,
	Dbvt_IWriter_WriteNode = CLIB.btDbvt_IWriter_WriteNode,
	MultimaterialTriangleMeshShape_getMaterialProperties = CLIB.btMultimaterialTriangleMeshShape_getMaterialProperties,
	TypedConstraint_getJointFeedback = CLIB.btTypedConstraint_getJointFeedback,
	GjkPairDetector_new = CLIB.btGjkPairDetector_new,
	TypedConstraint_getInfo2 = CLIB.btTypedConstraint_getInfo2,
	TypedConstraint_getInfo1 = CLIB.btTypedConstraint_getInfo1,
	MultibodyLink_setDVector = CLIB.btMultibodyLink_setDVector,
	GjkPairDetector_getClosestPointsNonVirtual = CLIB.btGjkPairDetector_getClosestPointsNonVirtual,
	CylinderShape_getRadius = CLIB.btCylinderShape_getRadius,
	VoronoiSimplexSolver_getCachedBC = CLIB.btVoronoiSimplexSolver_getCachedBC,
	CollisionWorld_LocalConvexResult_new = CLIB.btCollisionWorld_LocalConvexResult_new,
	CollisionWorld_ConvexResultCallbackWrapper_needsCollision = CLIB.btCollisionWorld_ConvexResultCallbackWrapper_needsCollision,
	TypedConstraint_getConstraintType = CLIB.btTypedConstraint_getConstraintType,
	StridingMeshInterface_getLockedReadOnlyVertexIndexBase = CLIB.btStridingMeshInterface_getLockedReadOnlyVertexIndexBase,
	JointFeedback_setAppliedForceBodyA = CLIB.btJointFeedback_setAppliedForceBodyA,
	TypedConstraint_getBreakingImpulseThreshold = CLIB.btTypedConstraint_getBreakingImpulseThreshold,
	TypedConstraint_getAppliedImpulse = CLIB.btTypedConstraint_getAppliedImpulse,
	MultiBody_getLinkInertia = CLIB.btMultiBody_getLinkInertia,
	DispatcherInfo_getEnableSPU = CLIB.btDispatcherInfo_getEnableSPU,
	RotationalLimitMotor_getHiLimit = CLIB.btRotationalLimitMotor_getHiLimit,
	TypedConstraint_calculateSerializeBufferSize = CLIB.btTypedConstraint_calculateSerializeBufferSize,
	SoftBody_setTotalDensity = CLIB.btSoftBody_setTotalDensity,
	SoftBody_Pose_getPos = CLIB.btSoftBody_Pose_getPos,
	TypedConstraint_buildJacobian = CLIB.btTypedConstraint_buildJacobian,
	TranslationalLimitMotor_getEnableMotor = CLIB.btTranslationalLimitMotor_getEnableMotor,
	MultiBody_addJointTorqueMultiDof2 = CLIB.btMultiBody_addJointTorqueMultiDof2,
	AABB_merge = CLIB.btAABB_merge,
	SoftBody_Tetra_getLeaf = CLIB.btSoftBody_Tetra_getLeaf,
	SoftBody_Tetra_getRv = CLIB.btSoftBody_Tetra_getRv,
	SoftBody_Material_getKLST = CLIB.btSoftBody_Material_getKLST,
	ConeShapeX_new = CLIB.btConeShapeX_new,
	RigidBody_getGravity = CLIB.btRigidBody_getGravity,
	TypedConstraint_btConstraintInfo2_delete = CLIB.btTypedConstraint_btConstraintInfo2_delete,
	TypedConstraint_btConstraintInfo2_setUpperLimit = CLIB.btTypedConstraint_btConstraintInfo2_setUpperLimit,
	TypedConstraint_btConstraintInfo2_setRowskip = CLIB.btTypedConstraint_btConstraintInfo2_setRowskip,
	TypedConstraint_btConstraintInfo2_setNumIterations = CLIB.btTypedConstraint_btConstraintInfo2_setNumIterations,
	TypedConstraint_btConstraintInfo2_setLowerLimit = CLIB.btTypedConstraint_btConstraintInfo2_setLowerLimit,
	SoftBody_sRayCast_getBody = CLIB.btSoftBody_sRayCast_getBody,
	TypedConstraint_btConstraintInfo2_setJ2linearAxis = CLIB.btTypedConstraint_btConstraintInfo2_setJ2linearAxis,
	MultiBody_setBaseVel = CLIB.btMultiBody_setBaseVel,
	TypedConstraint_btConstraintInfo2_setJ2angularAxis = CLIB.btTypedConstraint_btConstraintInfo2_setJ2angularAxis,
	TypedConstraint_btConstraintInfo2_setJ1linearAxis = CLIB.btTypedConstraint_btConstraintInfo2_setJ1linearAxis,
	TypedConstraint_btConstraintInfo2_setJ1angularAxis = CLIB.btTypedConstraint_btConstraintInfo2_setJ1angularAxis,
	CapsuleShape_getHalfHeight = CLIB.btCapsuleShape_getHalfHeight,
	TypedConstraint_btConstraintInfo2_setFps = CLIB.btTypedConstraint_btConstraintInfo2_setFps,
	CollisionObject_getBroadphaseHandle = CLIB.btCollisionObject_getBroadphaseHandle,
	TypedConstraint_btConstraintInfo2_setErp = CLIB.btTypedConstraint_btConstraintInfo2_setErp,
	TypedConstraint_btConstraintInfo2_setDamping = CLIB.btTypedConstraint_btConstraintInfo2_setDamping,
	AlignedObjectArray_btIndexedMesh_at = CLIB.btAlignedObjectArray_btIndexedMesh_at,
	WorldImporter_createScaledTrangleMeshShape = CLIB.btWorldImporter_createScaledTrangleMeshShape,
	TypedConstraint_btConstraintInfo2_setCfm = CLIB.btTypedConstraint_btConstraintInfo2_setCfm,
	TypedConstraint_btConstraintInfo2_getUpperLimit = CLIB.btTypedConstraint_btConstraintInfo2_getUpperLimit,
	CollisionShape_isSoftBody = CLIB.btCollisionShape_isSoftBody,
	ConvexPointCloudShape_getScaledPoint = CLIB.btConvexPointCloudShape_getScaledPoint,
	MultiBodyConstraint_getMaxAppliedImpulse = CLIB.btMultiBodyConstraint_getMaxAppliedImpulse,
	SoftBody_getAnchors = CLIB.btSoftBody_getAnchors,
	RotationalLimitMotor2_getHiLimit = CLIB.btRotationalLimitMotor2_getHiLimit,
	RotationalLimitMotor2_setEnableMotor = CLIB.btRotationalLimitMotor2_setEnableMotor,
	MultiBody_setBasePos = CLIB.btMultiBody_setBasePos,
	MultiBody_getVelocityVector = CLIB.btMultiBody_getVelocityVector,
	TypedConstraint_btConstraintInfo2_getJ2linearAxis = CLIB.btTypedConstraint_btConstraintInfo2_getJ2linearAxis,
	TypedConstraint_btConstraintInfo2_getJ2angularAxis = CLIB.btTypedConstraint_btConstraintInfo2_getJ2angularAxis,
	ConvexHullShape_getScaledPoint = CLIB.btConvexHullShape_getScaledPoint,
	GImpactBvh_delete = CLIB.btGImpactBvh_delete,
	TypedConstraint_btConstraintInfo2_getJ1angularAxis = CLIB.btTypedConstraint_btConstraintInfo2_getJ1angularAxis,
	Dispatcher_freeCollisionAlgorithm = CLIB.btDispatcher_freeCollisionAlgorithm,
	TypedConstraint_btConstraintInfo2_getFps = CLIB.btTypedConstraint_btConstraintInfo2_getFps,
	QuantizedBvh_isQuantized = CLIB.btQuantizedBvh_isQuantized,
	QuantizedBvh_calculateSerializeBufferSize = CLIB.btQuantizedBvh_calculateSerializeBufferSize,
	CollisionWorld_setDebugDrawer = CLIB.btCollisionWorld_setDebugDrawer,
	DynamicsWorld_clearForces = CLIB.btDynamicsWorld_clearForces,
	Dbvt_sStkNN_delete = CLIB.btDbvt_sStkNN_delete,
	TypedConstraint_btConstraintInfo2_new = CLIB.btTypedConstraint_btConstraintInfo2_new,
	DiscreteDynamicsWorld_getCollisionWorld = CLIB.btDiscreteDynamicsWorld_getCollisionWorld,
	SoftBody_appendNote5 = CLIB.btSoftBody_appendNote5,
	MultibodyLink_setFlags = CLIB.btMultibodyLink_setFlags,
	ManifoldPoint_setCombinedRestitution = CLIB.btManifoldPoint_setCombinedRestitution,
	CollisionObjectWrapper_getPartId = CLIB.btCollisionObjectWrapper_getPartId,
	CharacterControllerInterface_setVelocityForTimeInterval = CLIB.btCharacterControllerInterface_setVelocityForTimeInterval,
	ConvexCast_CastResult_DebugDraw = CLIB.btConvexCast_CastResult_DebugDraw,
	TypedConstraint_btConstraintInfo1_setNub = CLIB.btTypedConstraint_btConstraintInfo1_setNub,
	WheelInfoConstructionInfo_new = CLIB.btWheelInfoConstructionInfo_new,
	TypedConstraint_btConstraintInfo1_getNub = CLIB.btTypedConstraint_btConstraintInfo1_getNub,
	TypedConstraint_btConstraintInfo1_new = CLIB.btTypedConstraint_btConstraintInfo1_new,
	Box2dBox2dCollisionAlgorithm_new = CLIB.btBox2dBox2dCollisionAlgorithm_new,
	NNCGConstraintSolver_getOnlyForNoneContact = CLIB.btNNCGConstraintSolver_getOnlyForNoneContact,
	SoftBody_Element_getTag = CLIB.btSoftBody_Element_getTag,
	DefaultSoftBodySolver_copySoftBodyToVertexBuffer = CLIB.btDefaultSoftBodySolver_copySoftBodyToVertexBuffer,
	JointFeedback_setAppliedTorqueBodyA = CLIB.btJointFeedback_setAppliedTorqueBodyA,
	DbvtBroadphase_getDeferedcollide = CLIB.btDbvtBroadphase_getDeferedcollide,
	TranslationalLimitMotor2_getSpringDampingLimited = CLIB.btTranslationalLimitMotor2_getSpringDampingLimited,
	QuantizedBvh_deSerializeFloat = CLIB.btQuantizedBvh_deSerializeFloat,
	SoftBodyWorldInfo_setMaxDisplacement = CLIB.btSoftBodyWorldInfo_setMaxDisplacement,
	QuantizedBvhNode_getEscapeIndex = CLIB.btQuantizedBvhNode_getEscapeIndex,
	RotationalLimitMotor2_getEquilibriumPoint = CLIB.btRotationalLimitMotor2_getEquilibriumPoint,
	JointFeedback_getAppliedForceBodyA = CLIB.btJointFeedback_getAppliedForceBodyA,
	JointFeedback_new = CLIB.btJointFeedback_new,
	MultiBody_worldPosToLocal = CLIB.btMultiBody_worldPosToLocal,
	TetrahedronShapeEx_new = CLIB.btTetrahedronShapeEx_new,
	TriangleShapeEx_overlap_test_conservative = CLIB.btTriangleShapeEx_overlap_test_conservative,
	WorldImporter_createConvexHullShape = CLIB.btWorldImporter_createConvexHullShape,
	TriangleShapeEx_new3 = CLIB.btTriangleShapeEx_new3,
	CapsuleShapeZ_new = CLIB.btCapsuleShapeZ_new,
	TriangleShapeEx_new = CLIB.btTriangleShapeEx_new,
	ConvexTriangleCallback_new = CLIB.btConvexTriangleCallback_new,
	PrimitiveTriangle_delete = CLIB.btPrimitiveTriangle_delete,
	RigidBody_getAngularFactor = CLIB.btRigidBody_getAngularFactor,
	PrimitiveTriangle_setPlane = CLIB.btPrimitiveTriangle_setPlane,
	PrimitiveTriangle_setMargin = CLIB.btPrimitiveTriangle_setMargin,
	PrimitiveTriangle_setDummy = CLIB.btPrimitiveTriangle_setDummy,
	RigidBody_btRigidBodyConstructionInfo_getAdditionalDamping = CLIB.btRigidBody_btRigidBodyConstructionInfo_getAdditionalDamping,
	PrimitiveTriangle_overlap_test_conservative = CLIB.btPrimitiveTriangle_overlap_test_conservative,
	PrimitiveTriangle_getVertices = CLIB.btPrimitiveTriangle_getVertices,
	PrimitiveTriangle_getPlane = CLIB.btPrimitiveTriangle_getPlane,
	PrimitiveTriangle_getMargin = CLIB.btPrimitiveTriangle_getMargin,
	SoftBody_rayTest2 = CLIB.btSoftBody_rayTest2,
	TriangleCallback_delete = CLIB.btTriangleCallback_delete,
	RigidBody_btRigidBodyConstructionInfo_setAngularSleepingThreshold = CLIB.btRigidBody_btRigidBodyConstructionInfo_setAngularSleepingThreshold,
	ManifoldPoint_setContactMotion1 = CLIB.btManifoldPoint_setContactMotion1,
	SoftBody_sCti_getNormal = CLIB.btSoftBody_sCti_getNormal,
	ConvexTriangleCallback_clearCache = CLIB.btConvexTriangleCallback_clearCache,
	ContactSolverInfoData_getGlobalCfm = CLIB.btContactSolverInfoData_getGlobalCfm,
	CollisionObject_setCcdSweptSphereRadius = CLIB.btCollisionObject_setCcdSweptSphereRadius,
	PrimitiveTriangle_clip_triangle = CLIB.btPrimitiveTriangle_clip_triangle,
	GearConstraint_setAxisA = CLIB.btGearConstraint_setAxisA,
	CollisionWorld_AllHitsRayResultCallback_getHitFractions = CLIB.btCollisionWorld_AllHitsRayResultCallback_getHitFractions,
	PrimitiveTriangle_applyTransform = CLIB.btPrimitiveTriangle_applyTransform,
	RaycastVehicle_setSteeringValue = CLIB.btRaycastVehicle_setSteeringValue,
	CollisionDispatcher_getDispatcherFlags = CLIB.btCollisionDispatcher_getDispatcherFlags,
	PersistentManifold_replaceContactPoint = CLIB.btPersistentManifold_replaceContactPoint,
	TranslationalLimitMotor2_setCurrentLimitErrorHi = CLIB.btTranslationalLimitMotor2_setCurrentLimitErrorHi,
	Dbvt_sStkCLN_setParent = CLIB.btDbvt_sStkCLN_setParent,
	MultiBodySolverConstraint_setCfm = CLIB.btMultiBodySolverConstraint_setCfm,
	CollisionWorld_ClosestRayResultCallback_new = CLIB.btCollisionWorld_ClosestRayResultCallback_new,
	MinkowskiSumShape_setTransformB = CLIB.btMinkowskiSumShape_setTransformB,
	SoftBody_Body_getSoft = CLIB.btSoftBody_Body_getSoft,
	GImpactBvh_buildSet = CLIB.btGImpactBvh_buildSet,
	Dbvt_sStkNPS_new = CLIB.btDbvt_sStkNPS_new,
	RaycastVehicle_btVehicleTuning_setMaxSuspensionForce = CLIB.btRaycastVehicle_btVehicleTuning_setMaxSuspensionForce,
	MultiBody_setUseGyroTerm = CLIB.btMultiBody_setUseGyroTerm,
	Dispatcher_getManifoldByIndexInternal = CLIB.btDispatcher_getManifoldByIndexInternal,
	RigidBody_btRigidBodyConstructionInfo_setMotionState = CLIB.btRigidBody_btRigidBodyConstructionInfo_setMotionState,
	TriangleShape_getVertices1 = CLIB.btTriangleShape_getVertices1,
	Dispatcher_findAlgorithm2 = CLIB.btDispatcher_findAlgorithm2,
	TriangleShape_getVertexPtr = CLIB.btTriangleShape_getVertexPtr,
	SoftBody_Config_getKPR = CLIB.btSoftBody_Config_getKPR,
	TriangleShape_getPlaneEquation = CLIB.btTriangleShape_getPlaneEquation,
	PointCollector_getPointInWorld = CLIB.btPointCollector_getPointInWorld,
	TriangleShape_calcNormal = CLIB.btTriangleShape_calcNormal,
	HingeConstraint_setAxis = CLIB.btHingeConstraint_setAxis,
	TriangleShape_new2 = CLIB.btTriangleShape_new2,
	TriangleShape_new = CLIB.btTriangleShape_new,
	Box2dShape_getCentroid = CLIB.btBox2dShape_getCentroid,
	TriangleMeshShape_localGetSupportingVertexWithoutMargin = CLIB.btTriangleMeshShape_localGetSupportingVertexWithoutMargin,
	GImpactCollisionAlgorithm_setPart0 = CLIB.btGImpactCollisionAlgorithm_setPart0,
	CollisionObject_setContactProcessingThreshold = CLIB.btCollisionObject_setContactProcessingThreshold,
	SoftBody_checkLink = CLIB.btSoftBody_checkLink,
	SoftBody_Anchor_setC2 = CLIB.btSoftBody_Anchor_setC2,
	CollisionShape_getBoundingSphere = CLIB.btCollisionShape_getBoundingSphere,
	DispatcherInfo_setUseEpa = CLIB.btDispatcherInfo_setUseEpa,
	CollisionWorld_LocalConvexResult_setHitNormalLocal = CLIB.btCollisionWorld_LocalConvexResult_setHitNormalLocal,
	Generic6DofConstraint_calculateTransforms2 = CLIB.btGeneric6DofConstraint_calculateTransforms2,
	RigidBody_getFlags = CLIB.btRigidBody_getFlags,
	GImpactMeshShapePart_TrimeshPrimitiveManager_getMeshInterface = CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_getMeshInterface,
	SoftBody_Joint_setErp = CLIB.btSoftBody_Joint_setErp,
	TriangleMesh_getNumTriangles = CLIB.btTriangleMesh_getNumTriangles,
	TriangleMesh_findOrAddVertex = CLIB.btTriangleMesh_findOrAddVertex,
	RigidBody_getLinearDamping = CLIB.btRigidBody_getLinearDamping,
	TriangleMesh_addTriangleIndices = CLIB.btTriangleMesh_addTriangleIndices,
	MultiBodyConstraintSolver_new = CLIB.btMultiBodyConstraintSolver_new,
	CollisionConfiguration_getCollisionAlgorithmPool = CLIB.btCollisionConfiguration_getCollisionAlgorithmPool,
	SoftBody_Link_getRl = CLIB.btSoftBody_Link_getRl,
	Generic6DofConstraint_updateRHS = CLIB.btGeneric6DofConstraint_updateRHS,
	CollisionWorld_LocalConvexResult_setHitCollisionObject = CLIB.btCollisionWorld_LocalConvexResult_setHitCollisionObject,
	WorldImporter_getNumConstraints = CLIB.btWorldImporter_getNumConstraints,
	TriangleMesh_new2 = CLIB.btTriangleMesh_new2,
	TriangleMesh_new = CLIB.btTriangleMesh_new,
	BvhTree_get_node_pointer = CLIB.btBvhTree_get_node_pointer,
	ConstraintSetting_setImpulseClamp = CLIB.btConstraintSetting_setImpulseClamp,
	SoftBody_Config_getPiterations = CLIB.btSoftBody_Config_getPiterations,
	TriangleInfoMap_setPlanarEpsilon = CLIB.btTriangleInfoMap_setPlanarEpsilon,
	ManifoldPoint_getPositionWorldOnB = CLIB.btManifoldPoint_getPositionWorldOnB,
	StridingMeshInterface_unLockVertexBase = CLIB.btStridingMeshInterface_unLockVertexBase,
	GImpactMeshShape_getMeshInterface = CLIB.btGImpactMeshShape_getMeshInterface,
	MaterialProperties_getTriangleMaterialStride = CLIB.btMaterialProperties_getTriangleMaterialStride,
	TriangleInfoMap_setMaxEdgeAngleThreshold = CLIB.btTriangleInfoMap_setMaxEdgeAngleThreshold,
	TriangleInfoMap_setEdgeDistanceThreshold = CLIB.btTriangleInfoMap_setEdgeDistanceThreshold,
	GImpactBvh_setPrimitiveManager = CLIB.btGImpactBvh_setPrimitiveManager,
	SoftBodyConcaveCollisionAlgorithm_CreateFunc_new = CLIB.btSoftBodyConcaveCollisionAlgorithm_CreateFunc_new,
	TriangleInfoMap_setConvexEpsilon = CLIB.btTriangleInfoMap_setConvexEpsilon,
	TriangleInfoMap_serialize = CLIB.btTriangleInfoMap_serialize,
	CollisionObject_checkCollideWith = CLIB.btCollisionObject_checkCollideWith,
	AlignedObjectArray_btSoftBodyPtr_resizeNoInitialize = CLIB.btAlignedObjectArray_btSoftBodyPtr_resizeNoInitialize,
	DiscreteCollisionDetectorInterface_Result_delete = CLIB.btDiscreteCollisionDetectorInterface_Result_delete,
	RaycastVehicle_btVehicleTuning_setSuspensionDamping = CLIB.btRaycastVehicle_btVehicleTuning_setSuspensionDamping,
	TriangleInfoMap_getPlanarEpsilon = CLIB.btTriangleInfoMap_getPlanarEpsilon,
	TriangleInfoMap_getEqualVertexThreshold = CLIB.btTriangleInfoMap_getEqualVertexThreshold,
	TriangleInfoMap_getConvexEpsilon = CLIB.btTriangleInfoMap_getConvexEpsilon,
	TriangleIndexVertexMaterialArray_getLockedMaterialBase2 = CLIB.btTriangleIndexVertexMaterialArray_getLockedMaterialBase2,
	RigidBody_translate = CLIB.btRigidBody_translate,
	Generic6DofSpring2Constraint_calculateTransforms = CLIB.btGeneric6DofSpring2Constraint_calculateTransforms,
	SliderConstraint_setTargetAngMotorVelocity = CLIB.btSliderConstraint_setTargetAngMotorVelocity,
	TriangleInfo_delete = CLIB.btTriangleInfo_delete,
	WorldImporter_getNumCollisionShapes = CLIB.btWorldImporter_getNumCollisionShapes,
	TriangleInfo_setEdgeV2V0Angle = CLIB.btTriangleInfo_setEdgeV2V0Angle,
	TriangleInfo_setEdgeV1V2Angle = CLIB.btTriangleInfo_setEdgeV1V2Angle,
	ShapeHull_delete = CLIB.btShapeHull_delete,
	TriangleInfo_getFlags = CLIB.btTriangleInfo_getFlags,
	AlignedObjectArray_btPersistentManifoldPtr_size = CLIB.btAlignedObjectArray_btPersistentManifoldPtr_size,
	ManifoldPoint_new2 = CLIB.btManifoldPoint_new2,
	TriangleInfo_getEdgeV2V0Angle = CLIB.btTriangleInfo_getEdgeV2V0Angle,
	RigidBody_removeConstraintRef = CLIB.btRigidBody_removeConstraintRef,
	TriangleInfo_getEdgeV1V2Angle = CLIB.btTriangleInfo_getEdgeV1V2Angle,
	ContactSolverInfoData_delete = CLIB.btContactSolverInfoData_delete,
	SoftBody_RContact_getC0 = CLIB.btSoftBody_RContact_getC0,
	MultiBody_setNumLinks = CLIB.btMultiBody_setNumLinks,
	GImpactCompoundShape_new = CLIB.btGImpactCompoundShape_new,
	ContactSolverInfoData_getMinimumSolverBatchSize = CLIB.btContactSolverInfoData_getMinimumSolverBatchSize,
	TriangleInfo_getEdgeV0V1Angle = CLIB.btTriangleInfo_getEdgeV0V1Angle,
	TriangleInfo_new = CLIB.btTriangleInfo_new,
	BU_Simplex1to4_new3 = CLIB.btBU_Simplex1to4_new3,
	MultiBody_getRVector = CLIB.btMultiBody_getRVector,
	TriangleIndexVertexMaterialArray_getLockedReadOnlyMaterialBase2 = CLIB.btTriangleIndexVertexMaterialArray_getLockedReadOnlyMaterialBase2,
	QuantizedBvh_getSubtreeInfoArray = CLIB.btQuantizedBvh_getSubtreeInfoArray,
	TriangleInfoMap_calculateSerializeBufferSize = CLIB.btTriangleInfoMap_calculateSerializeBufferSize,
	AngularLimit_isLimit = CLIB.btAngularLimit_isLimit,
	TriangleIndexVertexMaterialArray_addMaterialProperties = CLIB.btTriangleIndexVertexMaterialArray_addMaterialProperties,
	StridingMeshInterface_InternalProcessAllTriangles = CLIB.btStridingMeshInterface_InternalProcessAllTriangles,
	TriangleIndexVertexMaterialArray_new = CLIB.btTriangleIndexVertexMaterialArray_new,
	MaterialProperties_delete = CLIB.btMaterialProperties_delete,
	SoftBody_Body_getRigid = CLIB.btSoftBody_Body_getRigid,
	MaterialProperties_setTriangleType = CLIB.btMaterialProperties_setTriangleType,
	OverlappingPairCache_cleanProxyFromPairs = CLIB.btOverlappingPairCache_cleanProxyFromPairs,
	PolarDecomposition_delete = CLIB.btPolarDecomposition_delete,
	BvhTree_setNodeBound = CLIB.btBvhTree_setNodeBound,
	ConvexCast_CastResult_setFraction = CLIB.btConvexCast_CastResult_setFraction,
	MaterialProperties_setTriangleMaterialsBase = CLIB.btMaterialProperties_setTriangleMaterialsBase,
	AlignedObjectArray_btSoftBody_JointPtr_size = CLIB.btAlignedObjectArray_btSoftBody_JointPtr_size,
	MultibodyLink_setAxisTop = CLIB.btMultibodyLink_setAxisTop,
	MaterialProperties_setNumTriangles = CLIB.btMaterialProperties_setNumTriangles,
	MaterialProperties_setNumMaterials = CLIB.btMaterialProperties_setNumMaterials,
	Point2PointConstraint_getUseSolveConstraintObsolete = CLIB.btPoint2PointConstraint_getUseSolveConstraintObsolete,
	RaycastVehicle_btVehicleTuning_getFrictionSlip = CLIB.btRaycastVehicle_btVehicleTuning_getFrictionSlip,
	MaterialProperties_setMaterialType = CLIB.btMaterialProperties_setMaterialType,
	MaterialProperties_setMaterialStride = CLIB.btMaterialProperties_setMaterialStride,
	Generic6DofConstraint_getCalculatedTransformB = CLIB.btGeneric6DofConstraint_getCalculatedTransformB,
	IDebugDrawWrapper_new = CLIB.btIDebugDrawWrapper_new,
	MaterialProperties_setMaterialBase = CLIB.btMaterialProperties_setMaterialBase,
	MaterialProperties_getTriangleType = CLIB.btMaterialProperties_getTriangleType,
	WorldImporter_getConstraintByIndex = CLIB.btWorldImporter_getConstraintByIndex,
	SequentialImpulseConstraintSolver_getRandSeed = CLIB.btSequentialImpulseConstraintSolver_getRandSeed,
	OverlapFilterCallback_delete = CLIB.btOverlapFilterCallback_delete,
	MaterialProperties_getMaterialStride = CLIB.btMaterialProperties_getMaterialStride,
	BroadphaseProxy_isNonMoving = CLIB.btBroadphaseProxy_isNonMoving,
	UnionFind_getElement = CLIB.btUnionFind_getElement,
	MaterialProperties_new = CLIB.btMaterialProperties_new,
	Dbvt_sStkNN_setA = CLIB.btDbvt_sStkNN_setA,
	ConeTwistConstraint_getMaxMotorImpulse = CLIB.btConeTwistConstraint_getMaxMotorImpulse,
	SoftBody_applyClusters = CLIB.btSoftBody_applyClusters,
	CollisionObject_getUserPointer = CLIB.btCollisionObject_getUserPointer,
	TriangleIndexVertexArray_addIndexedMesh2 = CLIB.btTriangleIndexVertexArray_addIndexedMesh2,
	TriangleIndexVertexArray_addIndexedMesh = CLIB.btTriangleIndexVertexArray_addIndexedMesh,
	RotationalLimitMotor2_setMaxMotorForce = CLIB.btRotationalLimitMotor2_setMaxMotorForce,
	RaycastVehicle_updateWheelTransformsWS = CLIB.btRaycastVehicle_updateWheelTransformsWS,
	SoftBody_Config_getDsequence = CLIB.btSoftBody_Config_getDsequence,
	IndexedMesh_setVertexStride = CLIB.btIndexedMesh_setVertexStride,
	CollisionWorld_getNumCollisionObjects = CLIB.btCollisionWorld_getNumCollisionObjects,
	MultiBodyDynamicsWorld_removeMultiBody = CLIB.btMultiBodyDynamicsWorld_removeMultiBody,
	IndexedMesh_setTriangleIndexStride = CLIB.btIndexedMesh_setTriangleIndexStride,
	IndexedMesh_setTriangleIndexBase = CLIB.btIndexedMesh_setTriangleIndexBase,
	IndexedMesh_setNumVertices = CLIB.btIndexedMesh_setNumVertices,
	IndexedMesh_setNumTriangles = CLIB.btIndexedMesh_setNumTriangles,
	HingeConstraint_getMaxMotorImpulse = CLIB.btHingeConstraint_getMaxMotorImpulse,
	HingeConstraint_hasLimit = CLIB.btHingeConstraint_hasLimit,
	Dispatcher_releaseManifold = CLIB.btDispatcher_releaseManifold,
	GImpactShapeInterface_needsRetrieveTetrahedrons = CLIB.btGImpactShapeInterface_needsRetrieveTetrahedrons,
	ShapeHull_getIndexPointer = CLIB.btShapeHull_getIndexPointer,
	TranslationalLimitMotor_new = CLIB.btTranslationalLimitMotor_new,
	MultiBody_getLinkForce = CLIB.btMultiBody_getLinkForce,
	IndexedMesh_getVertexBase = CLIB.btIndexedMesh_getVertexBase,
	IndexedMesh_getTriangleIndexStride = CLIB.btIndexedMesh_getTriangleIndexStride,
	IndexedMesh_getTriangleIndexBase = CLIB.btIndexedMesh_getTriangleIndexBase,
	GjkPairDetector_getCachedSeparatingAxis = CLIB.btGjkPairDetector_getCachedSeparatingAxis,
	TranslationalLimitMotor2_setCurrentLimitError = CLIB.btTranslationalLimitMotor2_setCurrentLimitError,
	RigidBody_predictIntegratedTransform = CLIB.btRigidBody_predictIntegratedTransform,
	CollisionObject_setFriction = CLIB.btCollisionObject_setFriction,
	GImpactShapeInterface_getChildAabb = CLIB.btGImpactShapeInterface_getChildAabb,
	IndexedMesh_getNumVertices = CLIB.btIndexedMesh_getNumVertices,
	TranslationalLimitMotor2_getEnableSpring = CLIB.btTranslationalLimitMotor2_getEnableSpring,
	IndexedMesh_getNumTriangles = CLIB.btIndexedMesh_getNumTriangles,
	Generic6DofConstraint_calcAnchorPos = CLIB.btGeneric6DofConstraint_calcAnchorPos,
	GImpactBvh_getNodeCount = CLIB.btGImpactBvh_getNodeCount,
	ConstraintSetting_getImpulseClamp = CLIB.btConstraintSetting_getImpulseClamp,
	IndexedMesh_getIndexType = CLIB.btIndexedMesh_getIndexType,
	IndexedMesh_new = CLIB.btIndexedMesh_new,
	GjkPairDetector_getFixContactNormalDirection = CLIB.btGjkPairDetector_getFixContactNormalDirection,
	GImpactShapeInterface_hasBoxSet = CLIB.btGImpactShapeInterface_hasBoxSet,
	Dispatcher_getNewManifold = CLIB.btDispatcher_getNewManifold,
	ContactSolverInfoData_getSor = CLIB.btContactSolverInfoData_getSor,
	InternalTriangleIndexCallbackWrapper_new = CLIB.btInternalTriangleIndexCallbackWrapper_new,
	PrimitiveTriangle_get_edge_plane = CLIB.btPrimitiveTriangle_get_edge_plane,
	BroadphasePair_setAlgorithm = CLIB.btBroadphasePair_setAlgorithm,
	CollisionWorld_debugDrawObject = CLIB.btCollisionWorld_debugDrawObject,
	TriangleBuffer_getNumTriangles = CLIB.btTriangleBuffer_getNumTriangles,
	DispatcherInfo_getUseConvexConservativeDistanceUtil = CLIB.btDispatcherInfo_getUseConvexConservativeDistanceUtil,
	TranslationalLimitMotor_setCurrentLimitError = CLIB.btTranslationalLimitMotor_setCurrentLimitError,
	Triangle_delete = CLIB.btTriangle_delete,
	Triangle_setVertex2 = CLIB.btTriangle_setVertex2,
	RigidBody_applyCentralForce = CLIB.btRigidBody_applyCentralForce,
	Triangle_setVertex1 = CLIB.btTriangle_setVertex1,
	Triangle_setVertex0 = CLIB.btTriangle_setVertex0,
	HashedOverlappingPairCache_needsBroadphaseCollision = CLIB.btHashedOverlappingPairCache_needsBroadphaseCollision,
	GhostObject_rayTest = CLIB.btGhostObject_rayTest,
	HeightfieldTerrainShape_setUseDiamondSubdivision = CLIB.btHeightfieldTerrainShape_setUseDiamondSubdivision,
	Triangle_setTriangleIndex = CLIB.btTriangle_setTriangleIndex,
	QuantizedBvhTree_isLeafNode = CLIB.btQuantizedBvhTree_isLeafNode,
	Triangle_setPartId = CLIB.btTriangle_setPartId,
	SoftBodyHelpers_DrawNodeTree3 = CLIB.btSoftBodyHelpers_DrawNodeTree3,
	SoftBody_AJoint_Specs_setAxis = CLIB.btSoftBody_AJoint_Specs_setAxis,
	PersistentManifold_delete = CLIB.btPersistentManifold_delete,
	Triangle_getTriangleIndex = CLIB.btTriangle_getTriangleIndex,
	Triangle_getPartId = CLIB.btTriangle_getPartId,
	AlignedObjectArray_btSoftBody_Link_at = CLIB.btAlignedObjectArray_btSoftBody_Link_at,
	MultiBody_delete = CLIB.btMultiBody_delete,
	SoftBody_getTetras = CLIB.btSoftBody_getTetras,
	HeightfieldTerrainShape_setUseZigzagSubdivision = CLIB.btHeightfieldTerrainShape_setUseZigzagSubdivision,
	CollisionAlgorithmCreateFunc_delete = CLIB.btCollisionAlgorithmCreateFunc_delete,
	SoftBody_Config_getKDF = CLIB.btSoftBody_Config_getKDF,
	RigidBody_applyImpulse = CLIB.btRigidBody_applyImpulse,
	ConvexSeparatingDistanceUtil_updateSeparatingDistance = CLIB.btConvexSeparatingDistanceUtil_updateSeparatingDistance,
	CollisionWorld_RayResultCallbackWrapper_new = CLIB.btCollisionWorld_RayResultCallbackWrapper_new,
	ConvexSeparatingDistanceUtil_initSeparatingDistance = CLIB.btConvexSeparatingDistanceUtil_initSeparatingDistance,
	WorldImporter_createCylinderShapeZ = CLIB.btWorldImporter_createCylinderShapeZ,
	RigidBody_setFlags = CLIB.btRigidBody_setFlags,
	RigidBody_computeAngularImpulseDenominator = CLIB.btRigidBody_computeAngularImpulseDenominator,
	SoftBody_Cluster_getFramexform = CLIB.btSoftBody_Cluster_getFramexform,
	SoftBody_Impulse_getDrift = CLIB.btSoftBody_Impulse_getDrift,
	GhostPairCallback_new = CLIB.btGhostPairCallback_new,
	TransformUtil_calculateVelocityQuaternion = CLIB.btTransformUtil_calculateVelocityQuaternion,
	ConeTwistConstraint_getBiasFactor = CLIB.btConeTwistConstraint_getBiasFactor,
	SoftBody_Body_activate = CLIB.btSoftBody_Body_activate,
	ConvexCast_CastResult_setAllowedPenetration = CLIB.btConvexCast_CastResult_setAllowedPenetration,
	ManifoldPoint_setContactMotion2 = CLIB.btManifoldPoint_setContactMotion2,
	StridingMeshInterface_getLockedVertexIndexBase2 = CLIB.btStridingMeshInterface_getLockedVertexIndexBase2,
	SoftBody_Cluster_getLv = CLIB.btSoftBody_Cluster_getLv,
	SoftBody_Joint_setSdrift = CLIB.btSoftBody_Joint_setSdrift,
	BU_Simplex1to4_addVertex = CLIB.btBU_Simplex1to4_addVertex,
	TranslationalLimitMotor_new2 = CLIB.btTranslationalLimitMotor_new2,
	RigidBody_setGravity = CLIB.btRigidBody_setGravity,
	BU_Simplex1to4_new5 = CLIB.btBU_Simplex1to4_new5,
	Convex2dConvex2dAlgorithm_new = CLIB.btConvex2dConvex2dAlgorithm_new,
	BU_Simplex1to4_new2 = CLIB.btBU_Simplex1to4_new2,
	SoftBody_Joint_Terminate = CLIB.btSoftBody_Joint_Terminate,
	SoftSoftCollisionAlgorithm_new2 = CLIB.btSoftSoftCollisionAlgorithm_new2,
	StridingMeshInterface_delete = CLIB.btStridingMeshInterface_delete,
	StridingMeshInterface_setScaling = CLIB.btStridingMeshInterface_setScaling,
	Dbvt_optimizeBottomUp = CLIB.btDbvt_optimizeBottomUp,
	StridingMeshInterface_setPremadeAabb = CLIB.btStridingMeshInterface_setPremadeAabb,
	StridingMeshInterface_serialize = CLIB.btStridingMeshInterface_serialize,
	SoftBody_Cluster_getLocii = CLIB.btSoftBody_Cluster_getLocii,
	StridingMeshInterface_preallocateVertices = CLIB.btStridingMeshInterface_preallocateVertices,
	BulletWorldImporter_convertAllObjects = CLIB.btBulletWorldImporter_convertAllObjects,
	StridingMeshInterface_preallocateIndices = CLIB.btStridingMeshInterface_preallocateIndices,
	SliderConstraint_getAncorInB = CLIB.btSliderConstraint_getAncorInB,
	GImpactQuantizedBvh_getRightNode = CLIB.btGImpactQuantizedBvh_getRightNode,
	DynamicsWorld_stepSimulation2 = CLIB.btDynamicsWorld_stepSimulation2,
	StridingMeshInterface_hasPremadeAabb = CLIB.btStridingMeshInterface_hasPremadeAabb,
	StridingMeshInterface_getScaling = CLIB.btStridingMeshInterface_getScaling,
	StridingMeshInterface_getNumSubParts = CLIB.btStridingMeshInterface_getNumSubParts,
	MultibodyLink_setJointDamping = CLIB.btMultibodyLink_setJointDamping,
	SoftBody_addVelocity2 = CLIB.btSoftBody_addVelocity2,
	TransformUtil_calculateDiffAxisAngleQuaternion = CLIB.btTransformUtil_calculateDiffAxisAngleQuaternion,
	StridingMeshInterface_getLockedVertexIndexBase = CLIB.btStridingMeshInterface_getLockedVertexIndexBase,
	ConvexHullShape_new4 = CLIB.btConvexHullShape_new4,
	Generic6DofConstraint_getRotationalLimitMotor = CLIB.btGeneric6DofConstraint_getRotationalLimitMotor,
	SoftBody_Element_delete = CLIB.btSoftBody_Element_delete,
	MultibodyLink_updateCacheMultiDof2 = CLIB.btMultibodyLink_updateCacheMultiDof2,
	PersistentManifold_getCacheEntry = CLIB.btPersistentManifold_getCacheEntry,
	StaticPlaneShape_getPlaneNormal = CLIB.btStaticPlaneShape_getPlaneNormal,
	GImpactMeshShape_getMeshPartCount = CLIB.btGImpactMeshShape_getMeshPartCount,
	StaticPlaneShape_getPlaneConstant = CLIB.btStaticPlaneShape_getPlaneConstant,
	TypedConstraint_btConstraintInfo2_getNumIterations = CLIB.btTypedConstraint_btConstraintInfo2_getNumIterations,
	SphereTriangleCollisionAlgorithm_new2 = CLIB.btSphereTriangleCollisionAlgorithm_new2,
	PersistentManifold_getCompanionIdB = CLIB.btPersistentManifold_getCompanionIdB,
	DiscreteDynamicsWorld_debugDrawConstraint = CLIB.btDiscreteDynamicsWorld_debugDrawConstraint,
	SoftBody_Tetra_setC1 = CLIB.btSoftBody_Tetra_setC1,
	KinematicCharacterController_setUpAxis = CLIB.btKinematicCharacterController_setUpAxis,
	SliderConstraint_getRestitutionDirLin = CLIB.btSliderConstraint_getRestitutionDirLin,
	SoftBody_Pose_getBvolume = CLIB.btSoftBody_Pose_getBvolume,
	SoftBody_Node_getBattach = CLIB.btSoftBody_Node_getBattach,
	SoftBody_appendNote3 = CLIB.btSoftBody_appendNote3,
	SphereSphereCollisionAlgorithm_CreateFunc_new = CLIB.btSphereSphereCollisionAlgorithm_CreateFunc_new,
	SoftBody_appendNote2 = CLIB.btSoftBody_appendNote2,
	HingeConstraint_getLowerLimit = CLIB.btHingeConstraint_getLowerLimit,
	GImpactMeshShapePart_TrimeshPrimitiveManager_getStride = CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_getStride,
	CollisionWorld_updateAabbs = CLIB.btCollisionWorld_updateAabbs,
	SphereBoxCollisionAlgorithm_getSpherePenetration = CLIB.btSphereBoxCollisionAlgorithm_getSpherePenetration,
	PrimitiveManagerBase_get_primitive_count = CLIB.btPrimitiveManagerBase_get_primitive_count,
	SoftBody_Material_getFlags = CLIB.btSoftBody_Material_getFlags,
	SoftBody_generateBendingConstraints = CLIB.btSoftBody_generateBendingConstraints,
	HingeConstraint_setLimit4 = CLIB.btHingeConstraint_setLimit4,
	SphereBoxCollisionAlgorithm_new = CLIB.btSphereBoxCollisionAlgorithm_new,
	ConvexTriangleCallback_getAabbMin = CLIB.btConvexTriangleCallback_getAabbMin,
	DbvtProxy_setLeaf = CLIB.btDbvtProxy_setLeaf,
	SphereBoxCollisionAlgorithm_CreateFunc_new = CLIB.btSphereBoxCollisionAlgorithm_CreateFunc_new,
	SoftBody_Joint_Specs_delete = CLIB.btSoftBody_Joint_Specs_delete,
	RotationalLimitMotor_getNormalCFM = CLIB.btRotationalLimitMotor_getNormalCFM,
	SoftBody_appendTetra = CLIB.btSoftBody_appendTetra,
	SparseSdf3_Reset = CLIB.btSparseSdf3_Reset,
	MinkowskiSumShape_setTransformA = CLIB.btMinkowskiSumShape_setTransformA,
	SparseSdf3_RemoveReferences = CLIB.btSparseSdf3_RemoveReferences,
	PrimitiveManagerBase_is_trimesh = CLIB.btPrimitiveManagerBase_is_trimesh,
	RigidBody_btRigidBodyConstructionInfo_getRollingFriction = CLIB.btRigidBody_btRigidBodyConstructionInfo_getRollingFriction,
	SparseSdf3_Initialize2 = CLIB.btSparseSdf3_Initialize2,
	SoftBody_Config_getCollisions = CLIB.btSoftBody_Config_getCollisions,
	ConeTwistConstraint_setLimit = CLIB.btConeTwistConstraint_setLimit,
	SoftBody_initializeClusters = CLIB.btSoftBody_initializeClusters,
	MultiBody_worldDirToLocal = CLIB.btMultiBody_worldDirToLocal,
	KinematicCharacterController_setJumpSpeed = CLIB.btKinematicCharacterController_setJumpSpeed,
	WorldImporter_getBvhByIndex = CLIB.btWorldImporter_getBvhByIndex,
	ManifoldPoint_getPartId0 = CLIB.btManifoldPoint_getPartId0,
	SoftSoftCollisionAlgorithm_new = CLIB.btSoftSoftCollisionAlgorithm_new,
	SoftSoftCollisionAlgorithm_CreateFunc_new = CLIB.btSoftSoftCollisionAlgorithm_CreateFunc_new,
	SoftRigidDynamicsWorld_setDrawFlags = CLIB.btSoftRigidDynamicsWorld_setDrawFlags,
	SoftRigidDynamicsWorld_getWorldInfo = CLIB.btSoftRigidDynamicsWorld_getWorldInfo,
	MultibodyLink_setCachedRotParentToThis = CLIB.btMultibodyLink_setCachedRotParentToThis,
	ConeTwistConstraint_isMaxMotorImpulseNormalized = CLIB.btConeTwistConstraint_isMaxMotorImpulseNormalized,
	AlignedObjectArray_btSoftBody_MaterialPtr_resizeNoInitialize = CLIB.btAlignedObjectArray_btSoftBody_MaterialPtr_resizeNoInitialize,
	SoftBody_SolverState_getIsdt = CLIB.btSoftBody_SolverState_getIsdt,
	CharacterControllerInterface_warp = CLIB.btCharacterControllerInterface_warp,
	RigidBody_setCenterOfMassTransform = CLIB.btRigidBody_setCenterOfMassTransform,
	CollisionWorld_removeCollisionObject = CLIB.btCollisionWorld_removeCollisionObject,
	SoftRigidDynamicsWorld_getDrawFlags = CLIB.btSoftRigidDynamicsWorld_getDrawFlags,
	HingeConstraint_getUseReferenceFrameA = CLIB.btHingeConstraint_getUseReferenceFrameA,
	PairSet_new = CLIB.btPairSet_new,
	GImpactCompoundShape_new2 = CLIB.btGImpactCompoundShape_new2,
	SliderConstraint_getLinDepth = CLIB.btSliderConstraint_getLinDepth,
	SoftRigidDynamicsWorld_addSoftBody2 = CLIB.btSoftRigidDynamicsWorld_addSoftBody2,
	Generic6DofSpring2Constraint_setServoTarget = CLIB.btGeneric6DofSpring2Constraint_setServoTarget,
	SoftRigidDynamicsWorld_getSoftBodyArray = CLIB.btSoftRigidDynamicsWorld_getSoftBodyArray,
	RaycastVehicle_btVehicleTuning_new = CLIB.btRaycastVehicle_btVehicleTuning_new,
	ManifoldPoint_getContactMotion1 = CLIB.btManifoldPoint_getContactMotion1,
	SoftRigidDynamicsWorld_new2 = CLIB.btSoftRigidDynamicsWorld_new2,
	GImpactShapeInterface_getBulletTriangle = CLIB.btGImpactShapeInterface_getBulletTriangle,
	SoftRigidCollisionAlgorithm_new = CLIB.btSoftRigidCollisionAlgorithm_new,
	SoftBodySolverOutput_delete = CLIB.btSoftBodySolverOutput_delete,
	SoftBody_Config_getKSSHR_CL = CLIB.btSoftBody_Config_getKSSHR_CL,
	Generic6DofSpring2Constraint_setServo = CLIB.btGeneric6DofSpring2Constraint_setServo,
	AlignedObjectArray_btSoftBody_MaterialPtr_push_back = CLIB.btAlignedObjectArray_btSoftBody_MaterialPtr_push_back,
	MultiBody_stepPositionsMultiDof3 = CLIB.btMultiBody_stepPositionsMultiDof3,
	SoftBodySolver_setNumberOfVelocityIterations = CLIB.btSoftBodySolver_setNumberOfVelocityIterations,
	SoftBodySolver_setNumberOfPositionIterations = CLIB.btSoftBodySolver_setNumberOfPositionIterations,
	SoftBodySolver_predictMotion = CLIB.btSoftBodySolver_predictMotion,
	WorldImporter_createCollisionObject = CLIB.btWorldImporter_createCollisionObject,
	Generic6DofConstraint_getFrameOffsetB = CLIB.btGeneric6DofConstraint_getFrameOffsetB,
	SoftBodySolver_getNumberOfVelocityIterations = CLIB.btSoftBodySolver_getNumberOfVelocityIterations,
	Face_new = CLIB.btFace_new,
	ConeTwistConstraint_getAFrame = CLIB.btConeTwistConstraint_getAFrame,
	SoftBodySolver_getNumberOfPositionIterations = CLIB.btSoftBodySolver_getNumberOfPositionIterations,
	SoftBodyHelpers_DrawClusterTree2 = CLIB.btSoftBodyHelpers_DrawClusterTree2,
	RotationalLimitMotor_testLimitValue = CLIB.btRotationalLimitMotor_testLimitValue,
	RigidBody_applyForce = CLIB.btRigidBody_applyForce,
	SoftBodySolver_checkInitialized = CLIB.btSoftBodySolver_checkInitialized,
	MultibodyLink_getAbsFrameLocVelocity = CLIB.btMultibodyLink_getAbsFrameLocVelocity,
	GjkPairDetector_getLastUsedMethod = CLIB.btGjkPairDetector_getLastUsedMethod,
	MultiBody_getParent = CLIB.btMultiBody_getParent,
	Triangle_getVertex1 = CLIB.btTriangle_getVertex1,
	DiscreteDynamicsWorld_setApplySpeculativeContactRestitution = CLIB.btDiscreteDynamicsWorld_setApplySpeculativeContactRestitution,
	RotationalLimitMotor_setMaxLimitForce = CLIB.btRotationalLimitMotor_setMaxLimitForce,
	SoftBodyHelpers_DrawNodeTree2 = CLIB.btSoftBodyHelpers_DrawNodeTree2,
	SoftBodyHelpers_DrawNodeTree = CLIB.btSoftBodyHelpers_DrawNodeTree,
	SoftBodyHelpers_DrawInfos = CLIB.btSoftBodyHelpers_DrawInfos,
	AxisSweep3_getOverlappingPairUserCallback = CLIB.btAxisSweep3_getOverlappingPairUserCallback,
	SoftBody_RContact_getC1 = CLIB.btSoftBody_RContact_getC1,
	SoftBodyHelpers_DrawFrame = CLIB.btSoftBodyHelpers_DrawFrame,
	RotationalLimitMotor2_setServoTarget = CLIB.btRotationalLimitMotor2_setServoTarget,
	SoftBodyHelpers_DrawFaceTree2 = CLIB.btSoftBodyHelpers_DrawFaceTree2,
	ManifoldPoint_setPositionWorldOnB = CLIB.btManifoldPoint_setPositionWorldOnB,
	CollisionShape_delete = CLIB.btCollisionShape_delete,
	CollisionConfiguration_getCollisionAlgorithmCreateFunc = CLIB.btCollisionConfiguration_getCollisionAlgorithmCreateFunc,
	CollisionShape_getLocalScaling = CLIB.btCollisionShape_getLocalScaling,
	SoftBodySolver_copyBackToSoftBodies2 = CLIB.btSoftBodySolver_copyBackToSoftBodies2,
	WheelInfo_getWheelDirectionCS = CLIB.btWheelInfo_getWheelDirectionCS,
	GImpactMeshShapePart_TrimeshPrimitiveManager_setPart = CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_setPart,
	SoftBodyHelpers_Draw2 = CLIB.btSoftBodyHelpers_Draw2,
	MultiBodySolverConstraint_getLowerLimit = CLIB.btMultiBodySolverConstraint_getLowerLimit,
	SoftBodyHelpers_CreatePatchUV = CLIB.btSoftBodyHelpers_CreatePatchUV,
	ConeTwistConstraint_setMotorTarget = CLIB.btConeTwistConstraint_setMotorTarget,
	CollisionObject_setUserIndex = CLIB.btCollisionObject_setUserIndex,
	UniversalConstraint_getAnchor = CLIB.btUniversalConstraint_getAnchor,
	TranslationalLimitMotor2_setStopERP = CLIB.btTranslationalLimitMotor2_setStopERP,
	SoftBody_getTetraVertexNormalData = CLIB.btSoftBody_getTetraVertexNormalData,
	OptimizedBvhNode_setSubPart = CLIB.btOptimizedBvhNode_setSubPart,
	CollisionWorld_rayTestSingle = CLIB.btCollisionWorld_rayTestSingle,
	BroadphaseProxy_delete = CLIB.btBroadphaseProxy_delete,
	SoftBody_getFaceVertexNormalData2 = CLIB.btSoftBody_getFaceVertexNormalData2,
	SoftBody_getFaceVertexNormalData = CLIB.btSoftBody_getFaceVertexNormalData,
	CollisionWorld_getDispatcher = CLIB.btCollisionWorld_getDispatcher,
	SoftBody_VSolve_Links = CLIB.btSoftBody_VSolve_Links,
	MultibodyLink_setMass = CLIB.btMultibodyLink_setMass,
	SoftBody_updateNormals = CLIB.btSoftBody_updateNormals,
	MultiBody_addBaseConstraintForce = CLIB.btMultiBody_addBaseConstraintForce,
	AABB_increment_margin = CLIB.btAABB_increment_margin,
	VoronoiSimplexSolver_closestPtPointTriangle = CLIB.btVoronoiSimplexSolver_closestPtPointTriangle,
	AlignedObjectArray_btPersistentManifoldPtr_new = CLIB.btAlignedObjectArray_btPersistentManifoldPtr_new,
	SoftBody_updateBounds = CLIB.btSoftBody_updateBounds,
	HingeConstraint_updateRHS = CLIB.btHingeConstraint_updateRHS,
	WheelInfo_RaycastInfo_getContactPointWS = CLIB.btWheelInfo_RaycastInfo_getContactPointWS,
	TranslationalLimitMotor2_getEnableMotor = CLIB.btTranslationalLimitMotor2_getEnableMotor,
	SoftBody_PSolve_Anchors = CLIB.btSoftBody_PSolve_Anchors,
	ContactSolverInfoData_setMaxGyroscopicForce = CLIB.btContactSolverInfoData_setMaxGyroscopicForce,
	RaycastVehicle_setUserConstraintType = CLIB.btRaycastVehicle_setUserConstraintType,
	ConvexPointCloudShape_getUnscaledPoints = CLIB.btConvexPointCloudShape_getUnscaledPoints,
	SoftBody_Config_setKVC = CLIB.btSoftBody_Config_setKVC,
	SoftBody_translate = CLIB.btSoftBody_translate,
	SoftBody_transform = CLIB.btSoftBody_transform,
	Generic6DofConstraint_getCalculatedTransformA = CLIB.btGeneric6DofConstraint_getCalculatedTransformA,
	SoftBody_Link_setC2 = CLIB.btSoftBody_Link_setC2,
	WheelInfo_getSuspensionStiffness = CLIB.btWheelInfo_getSuspensionStiffness,
	SoftBody_solveCommonConstraints = CLIB.btSoftBody_solveCommonConstraints,
	SoftBody_solveClusters2 = CLIB.btSoftBody_solveClusters2,
	MultiBody_getJointVelMultiDof = CLIB.btMultiBody_getJointVelMultiDof,
	SoftBody_setVolumeMass = CLIB.btSoftBody_setVolumeMass,
	SoftBody_setVolumeDensity = CLIB.btSoftBody_setVolumeDensity,
	SoftBody_setWindVelocity = CLIB.btSoftBody_setWindVelocity,
	SoftBody_setVelocity = CLIB.btSoftBody_setVelocity,
	MultiBody_getBaseVel = CLIB.btMultiBody_getBaseVel,
	SoftBody_scale = CLIB.btSoftBody_scale,
	SoftBody_setTotalMass = CLIB.btSoftBody_setTotalMass,
	OverlapCallback_processOverlap = CLIB.btOverlapCallback_processOverlap,
	OptimizedBvh_new = CLIB.btOptimizedBvh_new,
	SoftBody_setTag = CLIB.btSoftBody_setTag,
	SoftBody_setSolver = CLIB.btSoftBody_setSolver,
	VoronoiSimplexSolver_getLastW = CLIB.btVoronoiSimplexSolver_getLastW,
	SoftBody_setRestLengthScale = CLIB.btSoftBody_setRestLengthScale,
	SoftBody_setPose = CLIB.btSoftBody_setPose,
	GImpactMeshShape_new = CLIB.btGImpactMeshShape_new,
	SoftBody_setMass = CLIB.btSoftBody_setMass,
	TranslationalLimitMotor2_setBounce = CLIB.btTranslationalLimitMotor2_setBounce,
	SoftBody_setInitialWorldTransform = CLIB.btSoftBody_setInitialWorldTransform,
	DbvtProxy_getLeaf = CLIB.btDbvtProxy_getLeaf,
	SoftBody_setTotalMass2 = CLIB.btSoftBody_setTotalMass2,
	SoftBody_rotate = CLIB.btSoftBody_rotate,
	Dbvt_empty = CLIB.btDbvt_empty,
	CollisionObject_activate2 = CLIB.btCollisionObject_activate2,
	Dbvt_sStkNN_getA = CLIB.btDbvt_sStkNN_getA,
	SoftBody_resetLinkRestLengths = CLIB.btSoftBody_resetLinkRestLengths,
	BvhTree_getRightNode = CLIB.btBvhTree_getRightNode,
	MultiBody_getBaseOmega = CLIB.btMultiBody_getBaseOmega,
	MultibodyLink_getAppliedTorque = CLIB.btMultibodyLink_getAppliedTorque,
	SoftBody_releaseClusters = CLIB.btSoftBody_releaseClusters,
	SoftBody_releaseCluster = CLIB.btSoftBody_releaseCluster,
	MultiBody_addLinkTorque = CLIB.btMultiBody_addLinkTorque,
	SoftBody_refine = CLIB.btSoftBody_refine,
	PrimitiveTriangle_getDummy = CLIB.btPrimitiveTriangle_getDummy,
	RigidBody_computeImpulseDenominator = CLIB.btRigidBody_computeImpulseDenominator,
	SoftBody_rayTest = CLIB.btSoftBody_rayTest,
	RotationalLimitMotor2_setMotorERP = CLIB.btRotationalLimitMotor2_setMotorERP,
	BvhTriangleMeshShape_getOwnsBvh = CLIB.btBvhTriangleMeshShape_getOwnsBvh,
	SoftBody_Joint_Specs_new = CLIB.btSoftBody_Joint_Specs_new,
	SoftBody_PSolve_SContacts = CLIB.btSoftBody_PSolve_SContacts,
	SoftBody_PSolve_RContacts = CLIB.btSoftBody_PSolve_RContacts,
	MultiBody_setupPlanar2 = CLIB.btMultiBody_setupPlanar2,
	SoftBody_SContact_setNode = CLIB.btSoftBody_SContact_setNode,
	SoftBody_prepareClusters = CLIB.btSoftBody_prepareClusters,
	SoftBody_Note_setText = CLIB.btSoftBody_Note_setText,
	DiscreteCollisionDetectorInterface_ClosestPointInput_setMaximumDistanceSquared = CLIB.btDiscreteCollisionDetectorInterface_ClosestPointInput_setMaximumDistanceSquared,
	AlignedObjectArray_btSoftBody_JointPtr_at = CLIB.btAlignedObjectArray_btSoftBody_JointPtr_at,
	SliderConstraint_getDampingOrthoAng = CLIB.btSliderConstraint_getDampingOrthoAng,
	SoftBody_integrateMotion = CLIB.btSoftBody_integrateMotion,
	QuantizedBvhTree_get_node_pointer2 = CLIB.btQuantizedBvhTree_get_node_pointer2,
	GImpactMeshShapePart_getPart = CLIB.btGImpactMeshShapePart_getPart,
	AxisSweep3_quantize = CLIB.btAxisSweep3_quantize,
	SoftBody_initDefaults = CLIB.btSoftBody_initDefaults,
	BoxBoxDetector_getBox1 = CLIB.btBoxBoxDetector_getBox1,
	SoftBody_indicesToPointers = CLIB.btSoftBody_indicesToPointers,
	SoftBody_getVolume = CLIB.btSoftBody_getVolume,
	SoftBody_Anchor_setNode = CLIB.btSoftBody_Anchor_setNode,
	SoftBody_getUserIndexMapping = CLIB.btSoftBody_getUserIndexMapping,
	SoftBody_getTimeacc = CLIB.btSoftBody_getTimeacc,
	BvhTriangleMeshShape_new4 = CLIB.btBvhTriangleMeshShape_new4,
	GImpactMeshShapePart_new2 = CLIB.btGImpactMeshShapePart_new2,
	HingeConstraint_getMotorTargetVelosity = CLIB.btHingeConstraint_getMotorTargetVelosity,
	SoftBody_getTag = CLIB.btSoftBody_getTag,
	SimulationIslandManager_buildIslands = CLIB.btSimulationIslandManager_buildIslands,
	CharacterControllerInterface_setWalkDirection = CLIB.btCharacterControllerInterface_setWalkDirection,
	TranslationalLimitMotor2_getCurrentLinearDiff = CLIB.btTranslationalLimitMotor2_getCurrentLinearDiff,
	SoftBody_getSoftBodySolver = CLIB.btSoftBody_getSoftBodySolver,
	Box2dBox2dCollisionAlgorithm_CreateFunc_new = CLIB.btBox2dBox2dCollisionAlgorithm_CreateFunc_new,
	SoftBody_getScontacts = CLIB.btSoftBody_getScontacts,
	MultiBody_updateCollisionObjectWorldTransforms = CLIB.btMultiBody_updateCollisionObjectWorldTransforms,
	RotationalLimitMotor_isLimited = CLIB.btRotationalLimitMotor_isLimited,
	SoftBody_getRestLengthScale = CLIB.btSoftBody_getRestLengthScale,
	SoftBody_getRcontacts = CLIB.btSoftBody_getRcontacts,
	SoftBody_getPose = CLIB.btSoftBody_getPose,
	PolarDecomposition_new = CLIB.btPolarDecomposition_new,
	SoftBody_Joint_getErp = CLIB.btSoftBody_Joint_getErp,
	MultiBodySolverConstraint_getOrgDofIndex = CLIB.btMultiBodySolverConstraint_getOrgDofIndex,
	SoftBody_getNdbvt = CLIB.btSoftBody_getNdbvt,
	SoftBody_getMaterials = CLIB.btSoftBody_getMaterials,
	SoftBodyHelpers_DrawClusterTree = CLIB.btSoftBodyHelpers_DrawClusterTree,
	SoftBody_getJoints = CLIB.btSoftBody_getJoints,
	ConstraintSolver_solveGroup = CLIB.btConstraintSolver_solveGroup,
	MultibodyLink_setCachedRVector = CLIB.btMultibodyLink_setCachedRVector,
	CollisionObject_getUserIndex = CLIB.btCollisionObject_getUserIndex,
	SoftBody_getInitialWorldTransform = CLIB.btSoftBody_getInitialWorldTransform,
	Box2dShape_new2 = CLIB.btBox2dShape_new2,
	SoftBody_getFdbvt = CLIB.btSoftBody_getFdbvt,
	Convex2dShape_new = CLIB.btConvex2dShape_new,
	SoftBody_getCollisionDisabledObjects = CLIB.btSoftBody_getCollisionDisabledObjects,
	Dbvt_IWriter_delete = CLIB.btDbvt_IWriter_delete,
	PersistentManifold_getBody1 = CLIB.btPersistentManifold_getBody1,
	QuantizedBvhTree_clearNodes = CLIB.btQuantizedBvhTree_clearNodes,
	SoftBody_AJoint_Specs_setIcontrol = CLIB.btSoftBody_AJoint_Specs_setIcontrol,
	SoftBody_getBounds = CLIB.btSoftBody_getBounds,
	TypedConstraint_btConstraintInfo2_getRowskip = CLIB.btTypedConstraint_btConstraintInfo2_getRowskip,
	SoftBody_generateClusters2 = CLIB.btSoftBody_generateClusters2,
	Point2PointConstraint_getFlags = CLIB.btPoint2PointConstraint_getFlags,
	SoftBody_generateClusters = CLIB.btSoftBody_generateClusters,
	MultiBodyPoint2Point_new = CLIB.btMultiBodyPoint2Point_new,
	MultiBody_getJointVel = CLIB.btMultiBody_getJointVel,
	SoftBody_Link_new2 = CLIB.btSoftBody_Link_new2,
	SoftBody_generateBendingConstraints2 = CLIB.btSoftBody_generateBendingConstraints2,
	ConeTwistConstraint_getInfo1NonVirtual = CLIB.btConeTwistConstraint_getInfo1NonVirtual,
	CollisionShape_getMargin = CLIB.btCollisionShape_getMargin,
	SoftBody_CJoint_getLife = CLIB.btSoftBody_CJoint_getLife,
	Dbvt_sStkCLN_getParent = CLIB.btDbvt_sStkCLN_getParent,
	AlignedObjectArray_btIndexedMesh_push_back = CLIB.btAlignedObjectArray_btIndexedMesh_push_back,
	ConeTwistConstraint_setLimit5 = CLIB.btConeTwistConstraint_setLimit5,
	Generic6DofConstraint_getLinearUpperLimit = CLIB.btGeneric6DofConstraint_getLinearUpperLimit,
	SoftBody_appendAnchor6 = CLIB.btSoftBody_appendAnchor6,
	MultiBody_localPosToWorld = CLIB.btMultiBody_localPosToWorld,
	SoftBody_cutLink2 = CLIB.btSoftBody_cutLink2,
	ManifoldPoint_setPositionWorldOnA = CLIB.btManifoldPoint_setPositionWorldOnA,
	SoftBody_clusterVImpulse = CLIB.btSoftBody_clusterVImpulse,
	SoftBody_getSst = CLIB.btSoftBody_getSst,
	DynamicsWorld_removeAction = CLIB.btDynamicsWorld_removeAction,
	SoftBody_clusterVAImpulse = CLIB.btSoftBody_clusterVAImpulse,
	PointCollector_setNormalOnBInWorld = CLIB.btPointCollector_setNormalOnBInWorld,
	RaycastVehicle_getRigidBody = CLIB.btRaycastVehicle_getRigidBody,
	CharacterControllerInterface_reset = CLIB.btCharacterControllerInterface_reset,
	MultiBody_useRK4Integration = CLIB.btMultiBody_useRK4Integration,
	Dbvt_delete = CLIB.btDbvt_delete,
	Generic6DofConstraint_setUseSolveConstraintObsolete = CLIB.btGeneric6DofConstraint_setUseSolveConstraintObsolete,
	SoftBody_clusterDImpulse = CLIB.btSoftBody_clusterDImpulse,
	MultiBodyConstraint_jacobianB = CLIB.btMultiBodyConstraint_jacobianB,
	SoftBody_Pose_getWgh = CLIB.btSoftBody_Pose_getWgh,
	SoftBody_clusterDCImpulse = CLIB.btSoftBody_clusterDCImpulse,
	SoftBody_Config_getKVC = CLIB.btSoftBody_Config_getKVC,
	SoftBody_clusterDAImpulse = CLIB.btSoftBody_clusterDAImpulse,
	CollisionShape_isNonMoving = CLIB.btCollisionShape_isNonMoving,
	SoftBody_sCti_getColObj = CLIB.btSoftBody_sCti_getColObj,
	CollisionObject_getAnisotropicFriction = CLIB.btCollisionObject_getAnisotropicFriction,
	SoftBody_clusterCount = CLIB.btSoftBody_clusterCount,
	QuantizedBvhTree_getRightNode = CLIB.btQuantizedBvhTree_getRightNode,
	SoftBody_clusterCom2 = CLIB.btSoftBody_clusterCom2,
	WheelInfo_getSuspensionRestLength = CLIB.btWheelInfo_getSuspensionRestLength,
	OverlappingPairCache_sortOverlappingPairs = CLIB.btOverlappingPairCache_sortOverlappingPairs,
	MultiBodyConstraint_updateJacobianSizes = CLIB.btMultiBodyConstraint_updateJacobianSizes,
	GImpactQuantizedBvh_hasHierarchy = CLIB.btGImpactQuantizedBvh_hasHierarchy,
	AxisSweep3_unQuantize = CLIB.btAxisSweep3_unQuantize,
	SoftBody_cleanupClusters = CLIB.btSoftBody_cleanupClusters,
	SoftBody_AJoint_Specs_new = CLIB.btSoftBody_AJoint_Specs_new,
	SoftBody_checkLink2 = CLIB.btSoftBody_checkLink2,
	SoftBody_Anchor_getNode = CLIB.btSoftBody_Anchor_getNode,
	SoftBody_checkFace = CLIB.btSoftBody_checkFace,
	BroadphaseProxy_isPolyhedral = CLIB.btBroadphaseProxy_isPolyhedral,
	VoronoiSimplexSolver_setEqualVertexThreshold = CLIB.btVoronoiSimplexSolver_setEqualVertexThreshold,
	ManifoldPoint_getContactMotion2 = CLIB.btManifoldPoint_getContactMotion2,
	SoftBody_appendTetra3 = CLIB.btSoftBody_appendTetra3,
	GImpactMeshShapePart_TrimeshPrimitiveManager_setIndicestype = CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_setIndicestype,
	SoftBody_appendNote9 = CLIB.btSoftBody_appendNote9,
	TypedConstraint_btConstraintInfo1_delete = CLIB.btTypedConstraint_btConstraintInfo1_delete,
	SoftBody_appendNote4 = CLIB.btSoftBody_appendNote4,
	SphereSphereCollisionAlgorithm_new = CLIB.btSphereSphereCollisionAlgorithm_new,
	ManifoldPoint_setAppliedImpulseLateral2 = CLIB.btManifoldPoint_setAppliedImpulseLateral2,
	SoftBodyWorldInfo_getWater_normal = CLIB.btSoftBodyWorldInfo_getWater_normal,
	DispatcherInfo_setUseContinuous = CLIB.btDispatcherInfo_setUseContinuous,
	ConvexPolyhedron_testContainment = CLIB.btConvexPolyhedron_testContainment,
	SoftBody_appendLink9 = CLIB.btSoftBody_appendLink9,
	ConvexPolyhedron_getME = CLIB.btConvexPolyhedron_getME,
	SoftBody_appendLink6 = CLIB.btSoftBody_appendLink6,
	SoftBody_appendLink5 = CLIB.btSoftBody_appendLink5,
	SoftBody_appendLink4 = CLIB.btSoftBody_appendLink4,
	SoftBody_Link_setRl = CLIB.btSoftBody_Link_setRl,
	BoxShape_new3 = CLIB.btBoxShape_new3,
	SoftBody_appendLink2 = CLIB.btSoftBody_appendLink2,
	Dbvt_ICollide_AllLeaves = CLIB.btDbvt_ICollide_AllLeaves,
	SoftBody_Node_getX = CLIB.btSoftBody_Node_getX,
	ConvexInternalAabbCachingShape_recalcLocalAabb = CLIB.btConvexInternalAabbCachingShape_recalcLocalAabb,
	TriangleInfoMap_delete = CLIB.btTriangleInfoMap_delete,
	SoftBodyWorldInfo_setGravity = CLIB.btSoftBodyWorldInfo_setGravity,
	DiscreteCollisionDetectorInterface_getClosestPoints = CLIB.btDiscreteCollisionDetectorInterface_getClosestPoints,
	SoftBody_sRayCast_delete = CLIB.btSoftBody_sRayCast_delete,
	SoftBody_appendLinearJoint3 = CLIB.btSoftBody_appendLinearJoint3,
	RigidBody_computeGyroscopicImpulseImplicit_Body = CLIB.btRigidBody_computeGyroscopicImpulseImplicit_Body,
	SoftBody_updateArea2 = CLIB.btSoftBody_updateArea2,
	RigidBody_getTotalTorque = CLIB.btRigidBody_getTotalTorque,
	RaycastVehicle_rayCast = CLIB.btRaycastVehicle_rayCast,
	MultiBody_new = CLIB.btMultiBody_new,
	MultimaterialTriangleMeshShape_new2 = CLIB.btMultimaterialTriangleMeshShape_new2,
	SoftBody_appendFace3 = CLIB.btSoftBody_appendFace3,
	SoftBody_appendFace = CLIB.btSoftBody_appendFace,
	MLCPSolver_getNumFallbacks = CLIB.btMLCPSolver_getNumFallbacks,
	SoftBody_Cluster_setMaxSelfCollisionImpulse = CLIB.btSoftBody_Cluster_setMaxSelfCollisionImpulse,
	SoftBody_appendAngularJoint2 = CLIB.btSoftBody_appendAngularJoint2,
	WheelInfoConstructionInfo_getWheelDirectionCS = CLIB.btWheelInfoConstructionInfo_getWheelDirectionCS,
	DispatcherInfo_getEnableSatConvex = CLIB.btDispatcherInfo_getEnableSatConvex,
	SoftBody_dampClusters = CLIB.btSoftBody_dampClusters,
	Face_getPlane = CLIB.btFace_getPlane,
	DbvtProxy_getLinks = CLIB.btDbvtProxy_getLinks,
	DbvtNode_getParent = CLIB.btDbvtNode_getParent,
	SoftBody_Joint_Specs_getCfm = CLIB.btSoftBody_Joint_Specs_getCfm,
	SoftBody_appendAnchor2 = CLIB.btSoftBody_appendAnchor2,
	SoftBody_appendAnchor = CLIB.btSoftBody_appendAnchor,
	RigidBody_getLinearFactor = CLIB.btRigidBody_getLinearFactor,
	Generic6DofSpring2Constraint_setRotationOrder = CLIB.btGeneric6DofSpring2Constraint_setRotationOrder,
	SoftBody_Body_getCollisionObject = CLIB.btSoftBody_Body_getCollisionObject,
	CollisionWorld_LocalRayResult_getHitFraction = CLIB.btCollisionWorld_LocalRayResult_getHitFraction,
	CollisionObject_setWorldTransform = CLIB.btCollisionObject_setWorldTransform,
	GImpactQuantizedBvh_boxQueryTrans = CLIB.btGImpactQuantizedBvh_boxQueryTrans,
	BroadphaseProxy_setAabbMin = CLIB.btBroadphaseProxy_setAabbMin,
	BoxShape_getHalfExtentsWithoutMargin = CLIB.btBoxShape_getHalfExtentsWithoutMargin,
	SoftBody_new = CLIB.btSoftBody_new,
	SoftBody_Tetra_setRv = CLIB.btSoftBody_Tetra_setRv,
	SoftBody_Tetra_setLeaf = CLIB.btSoftBody_Tetra_setLeaf,
	SoftBody_Tetra_setC2 = CLIB.btSoftBody_Tetra_setC2,
	SphereTriangleCollisionAlgorithm_CreateFunc_new = CLIB.btSphereTriangleCollisionAlgorithm_CreateFunc_new,
	SoftBody_Body_delete = CLIB.btSoftBody_Body_delete,
	SoftBody_Tetra_getN = CLIB.btSoftBody_Tetra_getN,
	CollisionObjectWrapper_setParent = CLIB.btCollisionObjectWrapper_setParent,
	SoftBody_Tetra_getC1 = CLIB.btSoftBody_Tetra_getC1,
	SoftBody_Tetra_getC0 = CLIB.btSoftBody_Tetra_getC0,
	SoftBody_sRayCast_setIndex = CLIB.btSoftBody_sRayCast_setIndex,
	SoftBody_sRayCast_setFraction = CLIB.btSoftBody_sRayCast_setFraction,
	SoftBody_sRayCast_setFeature = CLIB.btSoftBody_sRayCast_setFeature,
	SoftBody_sRayCast_setBody = CLIB.btSoftBody_sRayCast_setBody,
	CharacterControllerInterface_playerStep = CLIB.btCharacterControllerInterface_playerStep,
	SoftBody_AJoint_IControl_delete = CLIB.btSoftBody_AJoint_IControl_delete,
	SoftBody_sRayCast_getIndex = CLIB.btSoftBody_sRayCast_getIndex,
	SoftBody_sRayCast_getFraction = CLIB.btSoftBody_sRayCast_getFraction,
	Chunk_setOldPtr = CLIB.btChunk_setOldPtr,
	IDebugDraw_delete = CLIB.btIDebugDraw_delete,
	SoftBody_Joint_getRefs = CLIB.btSoftBody_Joint_getRefs,
	MultiBodySolverConstraint_setFrictionIndex = CLIB.btMultiBodySolverConstraint_setFrictionIndex,
	SoftBody_SolverState_setVelmrg = CLIB.btSoftBody_SolverState_setVelmrg,
	SoftBody_SolverState_setUpdmrg = CLIB.btSoftBody_SolverState_setUpdmrg,
	ContactSolverInfoData_setSplitImpulsePenetrationThreshold = CLIB.btContactSolverInfoData_setSplitImpulsePenetrationThreshold,
	SoftBody_SolverState_setSdt = CLIB.btSoftBody_SolverState_setSdt,
	SoftBody_SolverState_setRadmrg = CLIB.btSoftBody_SolverState_setRadmrg,
	GImpactMeshShape_getMeshPart = CLIB.btGImpactMeshShape_getMeshPart,
	SoftBody_SolverState_setIsdt = CLIB.btSoftBody_SolverState_setIsdt,
	SoftBody_SolverState_getVelmrg = CLIB.btSoftBody_SolverState_getVelmrg,
	RotationalLimitMotor_setCurrentPosition = CLIB.btRotationalLimitMotor_setCurrentPosition,
	SoftBody_Pose_getAqq = CLIB.btSoftBody_Pose_getAqq,
	TypedConstraint_needsFeedback = CLIB.btTypedConstraint_needsFeedback,
	SoftBody_SolverState_getSdt = CLIB.btSoftBody_SolverState_getSdt,
	SoftBody_SolverState_getRadmrg = CLIB.btSoftBody_SolverState_getRadmrg,
	SoftBody_sCti_delete = CLIB.btSoftBody_sCti_delete,
	SoftBody_sCti_setOffset = CLIB.btSoftBody_sCti_setOffset,
	MultiBodyDynamicsWorld_getNumMultiBodyConstraints = CLIB.btMultiBodyDynamicsWorld_getNumMultiBodyConstraints,
	CapsuleShape_getUpAxis = CLIB.btCapsuleShape_getUpAxis,
	ContactSolverInfoData_getLinearSlop = CLIB.btContactSolverInfoData_getLinearSlop,
	ManifoldResult_calculateCombinedFriction = CLIB.btManifoldResult_calculateCombinedFriction,
	SoftBody_sCti_setNormal = CLIB.btSoftBody_sCti_setNormal,
	CompoundShape_getDynamicAabbTree = CLIB.btCompoundShape_getDynamicAabbTree,
	CollisionAlgorithmConstructionInfo_setDispatcher1 = CLIB.btCollisionAlgorithmConstructionInfo_setDispatcher1,
	SoftBody_sCti_setColObj = CLIB.btSoftBody_sCti_setColObj,
	GhostObject_removeOverlappingObjectInternal = CLIB.btGhostObject_removeOverlappingObjectInternal,
	ConvexConvexAlgorithm_CreateFunc_getPdSolver = CLIB.btConvexConvexAlgorithm_CreateFunc_getPdSolver,
	SoftBody_sCti_getOffset = CLIB.btSoftBody_sCti_getOffset,
	SoftBody_sCti_new = CLIB.btSoftBody_sCti_new,
	Dbvt_sStkCLN_delete = CLIB.btDbvt_sStkCLN_delete,
	SoftBody_Impulse_new = CLIB.btSoftBody_Impulse_new,
	VehicleRaycaster_btVehicleRaycasterResult_delete = CLIB.btVehicleRaycaster_btVehicleRaycasterResult_delete,
	SoftBody_SContact_setNormal = CLIB.btSoftBody_SContact_setNormal,
	Dispatcher_needsCollision = CLIB.btDispatcher_needsCollision,
	SoftBody_PSolve_Links = CLIB.btSoftBody_PSolve_Links,
	BroadphaseProxy_isCompound = CLIB.btBroadphaseProxy_isCompound,
	SoftBody_SContact_setMargin = CLIB.btSoftBody_SContact_setMargin,
	SoftBody_SContact_setFriction = CLIB.btSoftBody_SContact_setFriction,
	CollisionWorld_ContactResultCallbackWrapper_new = CLIB.btCollisionWorld_ContactResultCallbackWrapper_new,
	SoftBody_SContact_setFace = CLIB.btSoftBody_SContact_setFace,
	ConeTwistConstraint_getSolveSwingLimit = CLIB.btConeTwistConstraint_getSolveSwingLimit,
	MultiBodyConstraint_getMultiBodyA = CLIB.btMultiBodyConstraint_getMultiBodyA,
	SoftBody_CJoint_getRpos = CLIB.btSoftBody_CJoint_getRpos,
	SoftBody_SContact_getNormal = CLIB.btSoftBody_SContact_getNormal,
	RotationalLimitMotor_setStopERP = CLIB.btRotationalLimitMotor_setStopERP,
	SoftBody_SContact_getNode = CLIB.btSoftBody_SContact_getNode,
	TriangleMesh_getWeldingThreshold = CLIB.btTriangleMesh_getWeldingThreshold,
	SoftBody_SContact_getFriction = CLIB.btSoftBody_SContact_getFriction,
	SoftBody_SContact_getCfm = CLIB.btSoftBody_SContact_getCfm,
	MultiBody_addLinkConstraintForce = CLIB.btMultiBody_addLinkConstraintForce,
	AxisSweep3_updateHandle = CLIB.btAxisSweep3_updateHandle,
	SoftBody_Node_setQ = CLIB.btSoftBody_Node_setQ,
	SoftBody_RContact_setNode = CLIB.btSoftBody_RContact_setNode,
	HingeConstraint_setUseFrameOffset = CLIB.btHingeConstraint_setUseFrameOffset,
	SoftBody_RContact_setC4 = CLIB.btSoftBody_RContact_setC4,
	SoftBody_AJoint_IControl_Default = CLIB.btSoftBody_AJoint_IControl_Default,
	SoftBody_RContact_setC3 = CLIB.btSoftBody_RContact_setC3,
	SoftBody_RContact_setC2 = CLIB.btSoftBody_RContact_setC2,
	CollisionWorld_getDebugDrawer = CLIB.btCollisionWorld_getDebugDrawer,
	SoftBody_RContact_setC1 = CLIB.btSoftBody_RContact_setC1,
	SoftBody_RContact_setC0 = CLIB.btSoftBody_RContact_setC0,
	PolyhedralConvexShape_initializePolyhedralFeatures2 = CLIB.btPolyhedralConvexShape_initializePolyhedralFeatures2,
	MultibodyLink_getInertiaLocal = CLIB.btMultibodyLink_getInertiaLocal,
	SoftBody_RContact_getNode = CLIB.btSoftBody_RContact_getNode,
	SoftBody_RContact_getC3 = CLIB.btSoftBody_RContact_getC3,
	MultibodyLink_setAxisBottom2 = CLIB.btMultibodyLink_setAxisBottom2,
	SoftBody_RContact_getC2 = CLIB.btSoftBody_RContact_getC2,
	GImpactCollisionAlgorithm_gimpact_vs_shape = CLIB.btGImpactCollisionAlgorithm_gimpact_vs_shape,
	SoftBody_RayFromToCaster_setTests = CLIB.btSoftBody_RayFromToCaster_setTests,
	SoftBody_RayFromToCaster_setRayNormalizedDirection = CLIB.btSoftBody_RayFromToCaster_setRayNormalizedDirection,
	CollisionDispatcher_defaultNearCallback = CLIB.btCollisionDispatcher_defaultNearCallback,
	SoftBody_RayFromToCaster_setRayTo = CLIB.btSoftBody_RayFromToCaster_setRayTo,
	SoftBody_Face_getLeaf = CLIB.btSoftBody_Face_getLeaf,
	SoftBody_RayFromToCaster_setRayFrom = CLIB.btSoftBody_RayFromToCaster_setRayFrom,
	SoftBody_RayFromToCaster_setMint = CLIB.btSoftBody_RayFromToCaster_setMint,
	CollisionWorld_ConvexResultCallback_addSingleResult = CLIB.btCollisionWorld_ConvexResultCallback_addSingleResult,
	MLCPSolver_setNumFallbacks = CLIB.btMLCPSolver_setNumFallbacks,
	TranslationalLimitMotor_isLimited = CLIB.btTranslationalLimitMotor_isLimited,
	GjkPairDetector_setPenetrationDepthSolver = CLIB.btGjkPairDetector_setPenetrationDepthSolver,
	SoftBody_RayFromToCaster_setFace = CLIB.btSoftBody_RayFromToCaster_setFace,
	SoftBody_RayFromToCaster_rayFromToTriangle2 = CLIB.btSoftBody_RayFromToCaster_rayFromToTriangle2,
	SoftBody_RayFromToCaster_getTests = CLIB.btSoftBody_RayFromToCaster_getTests,
	SoftBody_RayFromToCaster_getRayTo = CLIB.btSoftBody_RayFromToCaster_getRayTo,
	Dbvt_write = CLIB.btDbvt_write,
	Point2PointConstraint_getPivotInA = CLIB.btPoint2PointConstraint_getPivotInA,
	SoftBody_RayFromToCaster_getMint = CLIB.btSoftBody_RayFromToCaster_getMint,
	SoftBody_Joint_Prepare = CLIB.btSoftBody_Joint_Prepare,
	BroadphasePair_new2 = CLIB.btBroadphasePair_new2,
	ConeTwistConstraint_getTwistAngle = CLIB.btConeTwistConstraint_getTwistAngle,
	CollisionWorld_ClosestConvexResultCallback_getConvexToWorld = CLIB.btCollisionWorld_ClosestConvexResultCallback_getConvexToWorld,
	SliderConstraint_getDampingLimAng = CLIB.btSliderConstraint_getDampingLimAng,
	SoftBody_Pose_setVolume = CLIB.btSoftBody_Pose_setVolume,
	SoftBody_Pose_setScl = CLIB.btSoftBody_Pose_setScl,
	SoftBody_Pose_setRot = CLIB.btSoftBody_Pose_setRot,
	SliderConstraint_setSoftnessLimLin = CLIB.btSliderConstraint_setSoftnessLimLin,
	SoftBody_Pose_setCom = CLIB.btSoftBody_Pose_setCom,
	SoftBody_Pose_setBvolume = CLIB.btSoftBody_Pose_setBvolume,
	ConeTwistConstraint_setLimit3 = CLIB.btConeTwistConstraint_setLimit3,
	SoftBody_Pose_setBframe = CLIB.btSoftBody_Pose_setBframe,
	SoftBody_Pose_setAqq = CLIB.btSoftBody_Pose_setAqq,
	DefaultMotionState_getStartWorldTrans = CLIB.btDefaultMotionState_getStartWorldTrans,
	SoftBody_Pose_getRot = CLIB.btSoftBody_Pose_getRot,
	SoftBody_Pose_getCom = CLIB.btSoftBody_Pose_getCom,
	SoftBody_Pose_getBframe = CLIB.btSoftBody_Pose_getBframe,
	SoftBody_predictMotion = CLIB.btSoftBody_predictMotion,
	SoftBody_Note_setRank = CLIB.btSoftBody_Note_setRank,
	BvhTriangleMeshShape_refitTree = CLIB.btBvhTriangleMeshShape_refitTree,
	AABB_setMin = CLIB.btAABB_setMin,
	SoftBody_Note_setOffset = CLIB.btSoftBody_Note_setOffset,
	SoftBody_Note_getText = CLIB.btSoftBody_Note_getText,
	ContactSolverInfoData_getErp2 = CLIB.btContactSolverInfoData_getErp2,
	CylinderShape_new2 = CLIB.btCylinderShape_new2,
	Generic6DofConstraint_getTranslationalLimitMotor = CLIB.btGeneric6DofConstraint_getTranslationalLimitMotor,
	RotationalLimitMotor2_getServoMotor = CLIB.btRotationalLimitMotor2_getServoMotor,
	SoftBody_Note_getOffset = CLIB.btSoftBody_Note_getOffset,
	SoftBody_Note_getNodes = CLIB.btSoftBody_Note_getNodes,
	Generic6DofConstraint_setUseLinearReferenceFrameA = CLIB.btGeneric6DofConstraint_setUseLinearReferenceFrameA,
	PairCachingGhostObject_new = CLIB.btPairCachingGhostObject_new,
	SoftBody_Node_setV = CLIB.btSoftBody_Node_setV,
	SoftBody_RContact_delete = CLIB.btSoftBody_RContact_delete,
	Box2dShape_new3 = CLIB.btBox2dShape_new3,
	SoftBody_Node_setLeaf = CLIB.btSoftBody_Node_setLeaf,
	SoftBody_Node_setIm = CLIB.btSoftBody_Node_setIm,
	CollisionWorld_ClosestConvexResultCallback_getConvexFromWorld = CLIB.btCollisionWorld_ClosestConvexResultCallback_getConvexFromWorld,
	SoftBody_Node_setF = CLIB.btSoftBody_Node_setF,
	DiscreteDynamicsWorld_setNumTasks = CLIB.btDiscreteDynamicsWorld_setNumTasks,
	ActionInterface_debugDraw = CLIB.btActionInterface_debugDraw,
	DiscreteDynamicsWorld_getSynchronizeAllMotionStates = CLIB.btDiscreteDynamicsWorld_getSynchronizeAllMotionStates,
	RigidBody_getAabb = CLIB.btRigidBody_getAabb,
	SoftBody_Node_setBattach = CLIB.btSoftBody_Node_setBattach,
	SoftBody_Node_setArea = CLIB.btSoftBody_Node_setArea,
	ConvexCast_CastResult_getAllowedPenetration = CLIB.btConvexCast_CastResult_getAllowedPenetration,
	ContactSolverInfoData_getRestingContactRestitutionThreshold = CLIB.btContactSolverInfoData_getRestingContactRestitutionThreshold,
	SoftBody_Node_getV = CLIB.btSoftBody_Node_getV,
	QuantizedBvhTree_new = CLIB.btQuantizedBvhTree_new,
	MultiBodySolverConstraint_getFrictionIndex = CLIB.btMultiBodySolverConstraint_getFrictionIndex,
	CompoundCollisionAlgorithm_CreateFunc_new = CLIB.btCompoundCollisionAlgorithm_CreateFunc_new,
	SoftBody_Node_getQ = CLIB.btSoftBody_Node_getQ,
	BroadphaseInterface_getOverlappingPairCache = CLIB.btBroadphaseInterface_getOverlappingPairCache,
	MultibodyLink_getPosVarCount = CLIB.btMultibodyLink_getPosVarCount,
	AngularLimit_getSign = CLIB.btAngularLimit_getSign,
	SoftBody_appendMaterial = CLIB.btSoftBody_appendMaterial,
	SoftBody_Node_getF = CLIB.btSoftBody_Node_getF,
	CollisionObjectWrapper_getWorldTransform = CLIB.btCollisionObjectWrapper_getWorldTransform,
	MultiBodyConstraint_debugDraw = CLIB.btMultiBodyConstraint_debugDraw,
	CompoundShape_updateChildTransform2 = CLIB.btCompoundShape_updateChildTransform2,
	SoftBody_Node_getArea = CLIB.btSoftBody_Node_getArea,
	SoftBody_Material_setKVST = CLIB.btSoftBody_Material_setKVST,
	MultiBody_fillConstraintJacobianMultiDof = CLIB.btMultiBody_fillConstraintJacobianMultiDof,
	CollisionWorld_LocalRayResult_getCollisionObject = CLIB.btCollisionWorld_LocalRayResult_getCollisionObject,
	SoftBody_Material_setKLST = CLIB.btSoftBody_Material_setKLST,
	QuantizedBvh_setQuantizationValues2 = CLIB.btQuantizedBvh_setQuantizationValues2,
	Generic6DofSpring2Constraint_calculateTransforms2 = CLIB.btGeneric6DofSpring2Constraint_calculateTransforms2,
	SoftBody_Material_setKAST = CLIB.btSoftBody_Material_setKAST,
	SoftBody_Material_setFlags = CLIB.btSoftBody_Material_setFlags,
	SoftBody_Material_getKVST = CLIB.btSoftBody_Material_getKVST,
	SphereBoxCollisionAlgorithm_getSphereDistance = CLIB.btSphereBoxCollisionAlgorithm_getSphereDistance,
	SoftBody_LJoint_getRpos = CLIB.btSoftBody_LJoint_getRpos,
	SoftBody_LJoint_Specs_setPosition = CLIB.btSoftBody_LJoint_Specs_setPosition,
	SoftBody_LJoint_Specs_getPosition = CLIB.btSoftBody_LJoint_Specs_getPosition,
	SoftBody_LJoint_Specs_new = CLIB.btSoftBody_LJoint_Specs_new,
	HingeConstraint_getFlags = CLIB.btHingeConstraint_getFlags,
	SoftBody_appendLink3 = CLIB.btSoftBody_appendLink3,
	GImpactBvh_getNodeData = CLIB.btGImpactBvh_getNodeData,
	SoftBody_Joint_Specs_getSplit = CLIB.btSoftBody_Joint_Specs_getSplit,
	SoftBody_Link_setC3 = CLIB.btSoftBody_Link_setC3,
	DbvtNode_isinternal = CLIB.btDbvtNode_isinternal,
	DynamicsWorld_getWorldUserInfo = CLIB.btDynamicsWorld_getWorldUserInfo,
	SoftBody_staticSolve = CLIB.btSoftBody_staticSolve,
	AlignedObjectArray_btVector3_push_back2 = CLIB.btAlignedObjectArray_btVector3_push_back2,
	WorldImporter_createPoint2PointConstraint2 = CLIB.btWorldImporter_createPoint2PointConstraint2,
	CollisionWorld_getDispatchInfo = CLIB.btCollisionWorld_getDispatchInfo,
	AngularLimit_delete = CLIB.btAngularLimit_delete,
	SoftBody_Link_getN = CLIB.btSoftBody_Link_getN,
	Generic6DofConstraint_setUseFrameOffset = CLIB.btGeneric6DofConstraint_setUseFrameOffset,
	MultiBodyJointLimitConstraint_new = CLIB.btMultiBodyJointLimitConstraint_new,
	NodeOverlapCallback_processNode = CLIB.btNodeOverlapCallback_processNode,
	SoftBody_Link_getC1 = CLIB.btSoftBody_Link_getC1,
	WorldImporter_createRigidBody = CLIB.btWorldImporter_createRigidBody,
	DbvtBroadphase_new2 = CLIB.btDbvtBroadphase_new2,
	TranslationalLimitMotor2_getLowerLimit = CLIB.btTranslationalLimitMotor2_getLowerLimit,
	Dbvt_optimizeIncremental = CLIB.btDbvt_optimizeIncremental,
	ConeShapeZ_new = CLIB.btConeShapeZ_new,
	SoftBody_Link_new = CLIB.btSoftBody_Link_new,
	BoxShape_new2 = CLIB.btBoxShape_new2,
	GImpactCollisionAlgorithm_registerAlgorithm = CLIB.btGImpactCollisionAlgorithm_registerAlgorithm,
	DispatcherInfo_getTimeOfImpact = CLIB.btDispatcherInfo_getTimeOfImpact,
	ConvexCast_CastResult_getDebugDrawer = CLIB.btConvexCast_CastResult_getDebugDrawer,
	SoftBody_Joint_delete = CLIB.btSoftBody_Joint_delete,
	SoftBody_Joint_Type = CLIB.btSoftBody_Joint_Type,
	GeometryUtil_isPointInsidePlanes = CLIB.btGeometryUtil_isPointInsidePlanes,
	SoftBody_Joint_Solve = CLIB.btSoftBody_Joint_Solve,
	ConvexConcaveCollisionAlgorithm_CreateFunc_new = CLIB.btConvexConcaveCollisionAlgorithm_CreateFunc_new,
	SoftBody_Joint_setSplit = CLIB.btSoftBody_Joint_setSplit,
	BvhTree_getEscapeNodeIndex = CLIB.btBvhTree_getEscapeNodeIndex,
	CollisionWorld_updateSingleAabb = CLIB.btCollisionWorld_updateSingleAabb,
	DbvtNode_new = CLIB.btDbvtNode_new,
	BU_Simplex1to4_reset = CLIB.btBU_Simplex1to4_reset,
	SoftBody_Joint_setMassmatrix = CLIB.btSoftBody_Joint_setMassmatrix,
	TriangleMesh_getUse32bitIndices = CLIB.btTriangleMesh_getUse32bitIndices,
	SoftBody_Joint_setDrift = CLIB.btSoftBody_Joint_setDrift,
	OptimizedBvh_updateBvhNodes = CLIB.btOptimizedBvh_updateBvhNodes,
	CollisionShape_getAnisotropicRollingFrictionDirection = CLIB.btCollisionShape_getAnisotropicRollingFrictionDirection,
	GImpactQuantizedBvh_getLeftNode = CLIB.btGImpactQuantizedBvh_getLeftNode,
	SoftBody_Joint_setDelete = CLIB.btSoftBody_Joint_setDelete,
	HingeConstraint_new6 = CLIB.btHingeConstraint_new6,
	SoftBody_Joint_setCfm = CLIB.btSoftBody_Joint_setCfm,
	PairSet_push_pair = CLIB.btPairSet_push_pair,
	SoftBody_RayFromToCaster_new = CLIB.btSoftBody_RayFromToCaster_new,
	SoftBody_Body_xform = CLIB.btSoftBody_Body_xform,
	MultibodyLink_getAppliedForce = CLIB.btMultibodyLink_getAppliedForce,
	SoftBody_Joint_getSplit = CLIB.btSoftBody_Joint_getSplit,
	SoftBody_Joint_getSdrift = CLIB.btSoftBody_Joint_getSdrift,
	Convex2dConvex2dAlgorithm_CreateFunc_setMinimumPointsPerturbationThreshold = CLIB.btConvex2dConvex2dAlgorithm_CreateFunc_setMinimumPointsPerturbationThreshold,
	GhostObject_new = CLIB.btGhostObject_new,
	SoftBody_Cluster_getAdamping = CLIB.btSoftBody_Cluster_getAdamping,
	SoftBody_Joint_getCfm = CLIB.btSoftBody_Joint_getCfm,
	SoftBody_Joint_getBodies = CLIB.btSoftBody_Joint_getBodies,
	BroadphasePair_getPProxy1 = CLIB.btBroadphasePair_getPProxy1,
	SoftBody_Joint_Specs_setSplit = CLIB.btSoftBody_Joint_Specs_setSplit,
	RaycastVehicle_updateWheelTransformsWS2 = CLIB.btRaycastVehicle_updateWheelTransformsWS2,
	SoftBody_Joint_Specs_setErp = CLIB.btSoftBody_Joint_Specs_setErp,
	MultibodyLink_setPosVarCount = CLIB.btMultibodyLink_setPosVarCount,
	GImpactMeshShapePart_TrimeshPrimitiveManager_getIndexstride = CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_getIndexstride,
	CollisionShape_getUserIndex = CLIB.btCollisionShape_getUserIndex,
	RigidBody_btRigidBodyConstructionInfo_getRestitution = CLIB.btRigidBody_btRigidBodyConstructionInfo_getRestitution,
	SoftBody_Config_getKCHR = CLIB.btSoftBody_Config_getKCHR,
	MultiBody_setCanSleep = CLIB.btMultiBody_setCanSleep,
	GhostObject_getNumOverlappingObjects = CLIB.btGhostObject_getNumOverlappingObjects,
	SoftBody_Joint_Specs_setCfm = CLIB.btSoftBody_Joint_Specs_setCfm,
	SoftBody_Joint_Specs_getErp = CLIB.btSoftBody_Joint_Specs_getErp,
	ConvexPolyhedron_getExtents = CLIB.btConvexPolyhedron_getExtents,
	SliderConstraint_setSoftnessDirLin = CLIB.btSliderConstraint_setSoftnessDirLin,
	ConvexPolyhedron_getUniqueEdges = CLIB.btConvexPolyhedron_getUniqueEdges,
	ConvexPointCloudShape_new2 = CLIB.btConvexPointCloudShape_new2,
	SoftBody_appendAnchor3 = CLIB.btSoftBody_appendAnchor3,
	RaycastVehicle_btVehicleTuning_delete = CLIB.btRaycastVehicle_btVehicleTuning_delete,
	CollisionShape_getName = CLIB.btCollisionShape_getName,
	SoftBody_Impulse_setVelocity = CLIB.btSoftBody_Impulse_setVelocity,
	SoftBody_Impulse_setDrift = CLIB.btSoftBody_Impulse_setDrift,
	DynamicsWorld_getSolverInfo = CLIB.btDynamicsWorld_getSolverInfo,
	SoftBody_Impulse_setAsVelocity = CLIB.btSoftBody_Impulse_setAsVelocity,
	SoftBody_Impulse_setAsDrift = CLIB.btSoftBody_Impulse_setAsDrift,
	SoftBody_updateLinkConstants = CLIB.btSoftBody_updateLinkConstants,
	SoftBody_Cluster_getMasses = CLIB.btSoftBody_Cluster_getMasses,
	TypedConstraint_btConstraintInfo2_getConstraintError = CLIB.btTypedConstraint_btConstraintInfo2_getConstraintError,
	MultiBodyLinkCollider_setMultiBody = CLIB.btMultiBodyLinkCollider_setMultiBody,
	ContinuousConvexCollision_new2 = CLIB.btContinuousConvexCollision_new2,
	SoftBody_SContact_delete = CLIB.btSoftBody_SContact_delete,
	SoftBody_ImplicitFn_delete = CLIB.btSoftBody_ImplicitFn_delete,
	SoftBody_ImplicitFn_Eval = CLIB.btSoftBody_ImplicitFn_Eval,
	BvhTree_build_tree = CLIB.btBvhTree_build_tree,
	SoftBody_Feature_setMaterial = CLIB.btSoftBody_Feature_setMaterial,
	SoftBody_Face_setRa = CLIB.btSoftBody_Face_setRa,
	BvhTree_getNodeCount = CLIB.btBvhTree_getNodeCount,
	SoftBody_Face_setNormal = CLIB.btSoftBody_Face_setNormal,
	Dbvt_setOpath = CLIB.btDbvt_setOpath,
	SoftBody_Face_setLeaf = CLIB.btSoftBody_Face_setLeaf,
	SoftBody_Face_getRa = CLIB.btSoftBody_Face_getRa,
	CharacterControllerInterface_canJump = CLIB.btCharacterControllerInterface_canJump,
	SoftBody_Face_getNormal = CLIB.btSoftBody_Face_getNormal,
	SoftBody_Face_getN = CLIB.btSoftBody_Face_getN,
	SoftBody_Element_setTag = CLIB.btSoftBody_Element_setTag,
	SoftBody_Config_setViterations = CLIB.btSoftBody_Config_setViterations,
	SoftBody_Config_setTimescale = CLIB.btSoftBody_Config_setTimescale,
	SoftBody_Config_setPiterations = CLIB.btSoftBody_Config_setPiterations,
	SoftBody_Cluster_getNodes = CLIB.btSoftBody_Cluster_getNodes,
	SoftBody_Config_setMaxvolume = CLIB.btSoftBody_Config_setMaxvolume,
	MultiBody_setBaseCollider = CLIB.btMultiBody_setBaseCollider,
	VoronoiSimplexSolver_setCachedValidClosest = CLIB.btVoronoiSimplexSolver_setCachedValidClosest,
	SoftBody_upcast = CLIB.btSoftBody_upcast,
	SoftBody_Config_setKSSHR_CL = CLIB.btSoftBody_Config_setKSSHR_CL,
	Dbvt_IClone_delete = CLIB.btDbvt_IClone_delete,
	SoftBody_Config_setKSS_SPLT_CL = CLIB.btSoftBody_Config_setKSS_SPLT_CL,
	SoftBody_Config_setKSRHR_CL = CLIB.btSoftBody_Config_setKSRHR_CL,
	SoftBody_Config_setKSR_SPLT_CL = CLIB.btSoftBody_Config_setKSR_SPLT_CL,
	DefaultCollisionConfiguration_setConvexConvexMultipointIterations = CLIB.btDefaultCollisionConfiguration_setConvexConvexMultipointIterations,
	SoftBody_Config_setKSKHR_CL = CLIB.btSoftBody_Config_setKSKHR_CL,
	Dbvt_sStkNPS_getNode = CLIB.btDbvt_sStkNPS_getNode,
	ManifoldPoint_getPositionWorldOnA = CLIB.btManifoldPoint_getPositionWorldOnA,
	SoftBody_Config_setKSK_SPLT_CL = CLIB.btSoftBody_Config_setKSK_SPLT_CL,
	SoftBody_Config_setKSHR = CLIB.btSoftBody_Config_setKSHR,
	SoftBody_Config_setKPR = CLIB.btSoftBody_Config_setKPR,
	SoftBody_Config_setKMT = CLIB.btSoftBody_Config_setKMT,
	SoftBody_Config_setKLF = CLIB.btSoftBody_Config_setKLF,
	SoftBody_Config_setKKHR = CLIB.btSoftBody_Config_setKKHR,
	SoftBody_Config_setKDP = CLIB.btSoftBody_Config_setKDP,
	SoftBody_Config_setKDG = CLIB.btSoftBody_Config_setKDG,
	SoftBody_Config_setKDF = CLIB.btSoftBody_Config_setKDF,
	ContactConstraint_getContactManifold = CLIB.btContactConstraint_getContactManifold,
	VoronoiSimplexSolver_getEqualVertexThreshold = CLIB.btVoronoiSimplexSolver_getEqualVertexThreshold,
	DbvtAabbMm_FromPoints2 = CLIB.btDbvtAabbMm_FromPoints2,
	Dispatcher_dispatchAllCollisionPairs = CLIB.btDispatcher_dispatchAllCollisionPairs,
	SoftBody_Config_setKAHR = CLIB.btSoftBody_Config_setKAHR,
	SoftBody_Config_setDiterations = CLIB.btSoftBody_Config_setDiterations,
	ConvexShape_getPreferredPenetrationDirection = CLIB.btConvexShape_getPreferredPenetrationDirection,
	RotationalLimitMotor2_setStopCFM = CLIB.btRotationalLimitMotor2_setStopCFM,
	DispatcherInfo_getDebugDraw = CLIB.btDispatcherInfo_getDebugDraw,
	RigidBody_getAngularVelocity = CLIB.btRigidBody_getAngularVelocity,
	SoftBody_Config_setCollisions = CLIB.btSoftBody_Config_setCollisions,
	MultiBodySolverConstraint_getOriginalContactPoint = CLIB.btMultiBodySolverConstraint_getOriginalContactPoint,
	SoftBody_Config_setCiterations = CLIB.btSoftBody_Config_setCiterations,
	SoftBody_Config_setAeromodel = CLIB.btSoftBody_Config_setAeromodel,
	SoftBody_Config_getVsequence = CLIB.btSoftBody_Config_getVsequence,
	ManifoldPoint_setContactPointFlags = CLIB.btManifoldPoint_setContactPointFlags,
	CollisionObject_setIgnoreCollisionCheck = CLIB.btCollisionObject_setIgnoreCollisionCheck,
	SoftBody_Config_getViterations = CLIB.btSoftBody_Config_getViterations,
	SliderConstraint_getAngDepth = CLIB.btSliderConstraint_getAngDepth,
	SoftBody_Config_getPsequence = CLIB.btSoftBody_Config_getPsequence,
	GjkPairDetector_setCachedSeparatingAxis = CLIB.btGjkPairDetector_setCachedSeparatingAxis,
	TriangleInfoMap_setZeroAreaThreshold = CLIB.btTriangleInfoMap_setZeroAreaThreshold,
	DispatcherInfo_getAllowedCcdPenetration = CLIB.btDispatcherInfo_getAllowedCcdPenetration,
	Vector3_array_set = CLIB.btVector3_array_set,
	DiscreteCollisionDetectorInterface_ClosestPointInput_setTransformA = CLIB.btDiscreteCollisionDetectorInterface_ClosestPointInput_setTransformA,
	SoftBodySolver_updateSoftBodies = CLIB.btSoftBodySolver_updateSoftBodies,
	SoftBody_Config_getKSS_SPLT_CL = CLIB.btSoftBody_Config_getKSS_SPLT_CL,
	SoftBody_Config_getKSRHR_CL = CLIB.btSoftBody_Config_getKSRHR_CL,
	TranslationalLimitMotor_getUpperLimit = CLIB.btTranslationalLimitMotor_getUpperLimit,
	SoftBody_Config_getKSR_SPLT_CL = CLIB.btSoftBody_Config_getKSR_SPLT_CL,
	ContactSolverInfoData_getSplitImpulseTurnErp = CLIB.btContactSolverInfoData_getSplitImpulseTurnErp,
	SoftBody_Config_getKSKHR_CL = CLIB.btSoftBody_Config_getKSKHR_CL,
	SoftBody_Config_getKSHR = CLIB.btSoftBody_Config_getKSHR,
	SoftBody_Config_getKMT = CLIB.btSoftBody_Config_getKMT,
	AngularLimit_getHigh = CLIB.btAngularLimit_getHigh,
	SoftBody_Config_getKKHR = CLIB.btSoftBody_Config_getKKHR,
	ConvexShape_getAabbNonVirtual = CLIB.btConvexShape_getAabbNonVirtual,
	AlignedObjectArray_btSoftBody_Node_size = CLIB.btAlignedObjectArray_btSoftBody_Node_size,
	WorldImporter_getRigidBodyByName = CLIB.btWorldImporter_getRigidBodyByName,
	SoftBody_Config_getDiterations = CLIB.btSoftBody_Config_getDiterations,
	MultibodyLink_getDVector = CLIB.btMultibodyLink_getDVector,
	CompoundShape_updateChildTransform = CLIB.btCompoundShape_updateChildTransform,
	DefaultCollisionConstructionInfo_getUseEpaPenetrationAlgorithm = CLIB.btDefaultCollisionConstructionInfo_getUseEpaPenetrationAlgorithm,
	DynamicsWorld_getWorldType = CLIB.btDynamicsWorld_getWorldType,
	Generic6DofSpring2Constraint_setDamping2 = CLIB.btGeneric6DofSpring2Constraint_setDamping2,
	MultiBody_checkMotionAndSleepIfRequired = CLIB.btMultiBody_checkMotionAndSleepIfRequired,
	SoftBody_Cluster_setSelfCollisionImpulseFactor = CLIB.btSoftBody_Cluster_setSelfCollisionImpulseFactor,
	HingeAccumulatedAngleConstraint_getAccumulatedHingeAngle = CLIB.btHingeAccumulatedAngleConstraint_getAccumulatedHingeAngle,
	SoftBody_Cluster_setNvimpulses = CLIB.btSoftBody_Cluster_setNvimpulses,
	RaycastVehicle_setUserConstraintId = CLIB.btRaycastVehicle_setUserConstraintId,
	SoftBody_Cluster_setNdimpulses = CLIB.btSoftBody_Cluster_setNdimpulses,
	AlignedObjectArray_btSoftBody_Node_push_back = CLIB.btAlignedObjectArray_btSoftBody_Node_push_back,
	RaycastVehicle_btVehicleTuning_getSuspensionDamping = CLIB.btRaycastVehicle_btVehicleTuning_getSuspensionDamping,
	CollisionWorld_ConvexResultCallback_setCollisionFilterGroup = CLIB.btCollisionWorld_ConvexResultCallback_setCollisionFilterGroup,
	SoftBody_Cluster_setNdamping = CLIB.btSoftBody_Cluster_setNdamping,
	GImpactShapeInterface_getLocalBox = CLIB.btGImpactShapeInterface_getLocalBox,
	SoftBody_appendAngularJoint4 = CLIB.btSoftBody_appendAngularJoint4,
	ConvexConvexAlgorithm_CreateFunc_setMinimumPointsPerturbationThreshold = CLIB.btConvexConvexAlgorithm_CreateFunc_setMinimumPointsPerturbationThreshold,
	SoftBody_Cluster_setMatching = CLIB.btSoftBody_Cluster_setMatching,
	SoftBody_Cluster_setLv = CLIB.btSoftBody_Cluster_setLv,
	TriangleMesh_new3 = CLIB.btTriangleMesh_new3,
	GImpactMeshShapePart_getVertex = CLIB.btGImpactMeshShapePart_getVertex,
	ConstraintSolver_prepareSolve = CLIB.btConstraintSolver_prepareSolve,
	GImpactCompoundShape_addChildShape = CLIB.btGImpactCompoundShape_addChildShape,
	SoftBody_Cluster_setLeaf = CLIB.btSoftBody_Cluster_setLeaf,
	Convex2dConvex2dAlgorithm_setLowLevelOfDetail = CLIB.btConvex2dConvex2dAlgorithm_setLowLevelOfDetail,
	SliderConstraint_setRestitutionDirAng = CLIB.btSliderConstraint_setRestitutionDirAng,
	SoftBody_Cluster_setLdamping = CLIB.btSoftBody_Cluster_setLdamping,
	SoftBody_Cluster_setInvwi = CLIB.btSoftBody_Cluster_setInvwi,
	MaterialProperties_getNumTriangles = CLIB.btMaterialProperties_getNumTriangles,
	GImpactShapeInterface_needsRetrieveTriangles = CLIB.btGImpactShapeInterface_needsRetrieveTriangles,
	WheelInfo_getWheelsDampingCompression = CLIB.btWheelInfo_getWheelsDampingCompression,
	SoftBody_Cluster_setFramexform = CLIB.btSoftBody_Cluster_setFramexform,
	MultiBody_getKineticEnergy = CLIB.btMultiBody_getKineticEnergy,
	SoftBody_Cluster_setCom = CLIB.btSoftBody_Cluster_setCom,
	SoftBody_Cluster_setCollide = CLIB.btSoftBody_Cluster_setCollide,
	BroadphaseProxy_isInfinite = CLIB.btBroadphaseProxy_isInfinite,
	MultiBodyJointMotor_setVelocityTarget = CLIB.btMultiBodyJointMotor_setVelocityTarget,
	MultibodyLink_getJointName = CLIB.btMultibodyLink_getJointName,
	SoftBody_Cluster_setClusterIndex = CLIB.btSoftBody_Cluster_setClusterIndex,
	SoftBody_Cluster_setAv = CLIB.btSoftBody_Cluster_setAv,
	SoftBody_Cluster_setAdamping = CLIB.btSoftBody_Cluster_setAdamping,
	RaycastVehicle_getChassisWorldTransform = CLIB.btRaycastVehicle_getChassisWorldTransform,
	SoftBody_Cluster_getVimpulses = CLIB.btSoftBody_Cluster_getVimpulses,
	ManifoldPoint_setPartId1 = CLIB.btManifoldPoint_setPartId1,
	CollisionWorld_getPairCache = CLIB.btCollisionWorld_getPairCache,
	SoftBody_Cluster_getNvimpulses = CLIB.btSoftBody_Cluster_getNvimpulses,
	TransformUtil_integrateTransform = CLIB.btTransformUtil_integrateTransform,
	SimulationIslandManager_storeIslandActivationState = CLIB.btSimulationIslandManager_storeIslandActivationState,
	SoftBody_Cluster_getNdamping = CLIB.btSoftBody_Cluster_getNdamping,
	SoftBody_Cluster_getMaxSelfCollisionImpulse = CLIB.btSoftBody_Cluster_getMaxSelfCollisionImpulse,
	SoftBody_Cluster_getMatching = CLIB.btSoftBody_Cluster_getMatching,
	TransformUtil_calculateDiffAxisAngle = CLIB.btTransformUtil_calculateDiffAxisAngle,
	CollisionWorld_AllHitsRayResultCallback_setRayFromWorld = CLIB.btCollisionWorld_AllHitsRayResultCallback_setRayFromWorld,
	GImpactBvh_get_node_pointer2 = CLIB.btGImpactBvh_get_node_pointer2,
	ConeTwistConstraint_calcAngleInfo2 = CLIB.btConeTwistConstraint_calcAngleInfo2,
	SoftBody_Cluster_getLdamping = CLIB.btSoftBody_Cluster_getLdamping,
	SoftBody_Cluster_getInvwi = CLIB.btSoftBody_Cluster_getInvwi,
	SoftBody_Cluster_getImass = CLIB.btSoftBody_Cluster_getImass,
	SoftBody_Cluster_getNdimpulses = CLIB.btSoftBody_Cluster_getNdimpulses,
	Generic6DofSpring2Constraint_setAxis = CLIB.btGeneric6DofSpring2Constraint_setAxis,
	SoftBody_Cluster_getDimpulses = CLIB.btSoftBody_Cluster_getDimpulses,
	MultiBody_addJointTorqueMultiDof = CLIB.btMultiBody_addJointTorqueMultiDof,
	MultiBodyConstraint_finalizeMultiDof = CLIB.btMultiBodyConstraint_finalizeMultiDof,
	MultibodyLink_setJointFriction = CLIB.btMultibodyLink_setJointFriction,
	RigidBody_clearForces = CLIB.btRigidBody_clearForces,
	SoftBody_Cluster_getCollide = CLIB.btSoftBody_Cluster_getCollide,
	CollisionWorld_ConvexResultCallback_delete = CLIB.btCollisionWorld_ConvexResultCallback_delete,
	SoftBody_Cluster_getClusterIndex = CLIB.btSoftBody_Cluster_getClusterIndex,
	CollisionWorld_LocalRayResult_delete = CLIB.btCollisionWorld_LocalRayResult_delete,
	SoftBody_Joint_getDrift = CLIB.btSoftBody_Joint_getDrift,
	SoftBody_CJoint_setNormal = CLIB.btSoftBody_CJoint_setNormal,
	DbvtBroadphase_getReleasepaircache = CLIB.btDbvtBroadphase_getReleasepaircache,
	SoftBody_CJoint_setMaxlife = CLIB.btSoftBody_CJoint_setMaxlife,
	TranslationalLimitMotor2_testLimitValue = CLIB.btTranslationalLimitMotor2_testLimitValue,
	PersistentManifold_getCompanionIdA = CLIB.btPersistentManifold_getCompanionIdA,
	SoftBody_CJoint_setLife = CLIB.btSoftBody_CJoint_setLife,
	SoftBody_CJoint_setFriction = CLIB.btSoftBody_CJoint_setFriction,
	RotationalLimitMotor_getStopCFM = CLIB.btRotationalLimitMotor_getStopCFM,
	SoftBody_CJoint_getNormal = CLIB.btSoftBody_CJoint_getNormal,
	SliderConstraint_setPoweredAngMotor = CLIB.btSliderConstraint_setPoweredAngMotor,
	SoftBody_Body_velocity = CLIB.btSoftBody_Body_velocity,
	SoftBody_Body_setSoft = CLIB.btSoftBody_Body_setSoft,
	SoftBody_Body_setRigid = CLIB.btSoftBody_Body_setRigid,
	Dbvt_getLkhd = CLIB.btDbvt_getLkhd,
	SoftBody_Body_setCollisionObject = CLIB.btSoftBody_Body_setCollisionObject,
	GImpactBvh_isLeafNode = CLIB.btGImpactBvh_isLeafNode,
	Dbvt_sStkCLN_getNode = CLIB.btDbvt_sStkCLN_getNode,
	SoftBody_Body_invWorldInertia = CLIB.btSoftBody_Body_invWorldInertia,
	GImpactShapeInterface_getNumChildShapes = CLIB.btGImpactShapeInterface_getNumChildShapes,
	SoftBody_Body_invMass = CLIB.btSoftBody_Body_invMass,
	SoftBody_addForce = CLIB.btSoftBody_addForce,
	Dbvt_IClone_new = CLIB.btDbvt_IClone_new,
	GImpactQuantizedBvh_get_node_pointer = CLIB.btGImpactQuantizedBvh_get_node_pointer,
	AngularLimit_getRelaxationFactor = CLIB.btAngularLimit_getRelaxationFactor,
	MultibodyLink_setAppliedForce = CLIB.btMultibodyLink_setAppliedForce,
	WorldImporter_deleteAllData = CLIB.btWorldImporter_deleteAllData,
	SoftBody_Body_applyImpulse = CLIB.btSoftBody_Body_applyImpulse,
	SoftBody_Body_applyDImpulse = CLIB.btSoftBody_Body_applyDImpulse,
	SoftBody_Body_applyDCImpulse = CLIB.btSoftBody_Body_applyDCImpulse,
	ManifoldPoint_new = CLIB.btManifoldPoint_new,
	SoftBody_Body_applyDAImpulse = CLIB.btSoftBody_Body_applyDAImpulse,
	HingeConstraint_setMaxMotorImpulse = CLIB.btHingeConstraint_setMaxMotorImpulse,
	WorldImporter_getNameForPointer = CLIB.btWorldImporter_getNameForPointer,
	TranslationalLimitMotor2_getMotorERP = CLIB.btTranslationalLimitMotor2_getMotorERP,
	SoftBody_Body_new3 = CLIB.btSoftBody_Body_new3,
	SoftBody_Body_new2 = CLIB.btSoftBody_Body_new2,
	CollisionObject_getCcdSweptSphereRadius = CLIB.btCollisionObject_getCcdSweptSphereRadius,
	AABB_getMin = CLIB.btAABB_getMin,
	SoftBody_Body_new = CLIB.btSoftBody_Body_new,
	SoftBody_getWindVelocity = CLIB.btSoftBody_getWindVelocity,
	SoftBody_Anchor_setLocal = CLIB.btSoftBody_Anchor_setLocal,
	SoftBody_Anchor_setInfluence = CLIB.btSoftBody_Anchor_setInfluence,
	TriangleMeshShape_getMeshInterface = CLIB.btTriangleMeshShape_getMeshInterface,
	RigidBody_btRigidBodyConstructionInfo_getFriction = CLIB.btRigidBody_btRigidBodyConstructionInfo_getFriction,
	WheelInfo_new = CLIB.btWheelInfo_new,
	RigidBody_getFrictionSolverType = CLIB.btRigidBody_getFrictionSolverType,
	SoftBody_Anchor_setC0 = CLIB.btSoftBody_Anchor_setC0,
	SoftBody_Anchor_setBody = CLIB.btSoftBody_Anchor_setBody,
	SoftBody_Anchor_getC2 = CLIB.btSoftBody_Anchor_getC2,
	HingeConstraint_setUseReferenceFrameA = CLIB.btHingeConstraint_setUseReferenceFrameA,
	ConvexConcaveCollisionAlgorithm_clearCache = CLIB.btConvexConcaveCollisionAlgorithm_clearCache,
	SoftBody_Anchor_getC1 = CLIB.btSoftBody_Anchor_getC1,
	Generic6DofSpring2Constraint_getTranslationalLimitMotor = CLIB.btGeneric6DofSpring2Constraint_getTranslationalLimitMotor,
	SoftBody_Anchor_getC0 = CLIB.btSoftBody_Anchor_getC0,
	BvhTriangleMeshShape_getTriangleInfoMap = CLIB.btBvhTriangleMeshShape_getTriangleInfoMap,
	CompoundShape_getChildList = CLIB.btCompoundShape_getChildList,
	ContactSolverInfoData_setNumIterations = CLIB.btContactSolverInfoData_setNumIterations,
	BvhTriangleMeshShape_buildOptimizedBvh = CLIB.btBvhTriangleMeshShape_buildOptimizedBvh,
	PersistentManifold_getContactProcessingThreshold = CLIB.btPersistentManifold_getContactProcessingThreshold,
	Generic6DofSpringConstraint_new2 = CLIB.btGeneric6DofSpringConstraint_new2,
	SoftBody_AJoint_setIcontrol = CLIB.btSoftBody_AJoint_setIcontrol,
	MultiBodySolverConstraint_getJacDiagABInv = CLIB.btMultiBodySolverConstraint_getJacDiagABInv,
	SoftBody_AJoint_getIcontrol = CLIB.btSoftBody_AJoint_getIcontrol,
	SoftBody_AJoint_getAxis = CLIB.btSoftBody_AJoint_getAxis,
	MultiBodySolverConstraint_getMultiBodyB = CLIB.btMultiBodySolverConstraint_getMultiBodyB,
	SoftBody_getBUpdateRtCst = CLIB.btSoftBody_getBUpdateRtCst,
	EmptyShape_new = CLIB.btEmptyShape_new,
	Triangle_getVertex0 = CLIB.btTriangle_getVertex0,
	SoftBody_AJoint_Specs_getIcontrol = CLIB.btSoftBody_AJoint_Specs_getIcontrol,
	SoftBody_AJoint_Specs_getAxis = CLIB.btSoftBody_AJoint_Specs_getAxis,
	Hinge2Constraint_getAngle2 = CLIB.btHinge2Constraint_getAngle2,
	SoftBody_AJoint_IControl_Prepare = CLIB.btSoftBody_AJoint_IControl_Prepare,
	SoftBody_Node_getN = CLIB.btSoftBody_Node_getN,
	DbvtNode_getDataAsInt = CLIB.btDbvtNode_getDataAsInt,
	DefaultVehicleRaycaster_new = CLIB.btDefaultVehicleRaycaster_new,
	SoftBody_AJoint_IControlWrapper_new = CLIB.btSoftBody_AJoint_IControlWrapper_new,
	SoftBodyWorldInfo_delete = CLIB.btSoftBodyWorldInfo_delete,
	SoftBodyWorldInfo_setWater_offset = CLIB.btSoftBodyWorldInfo_setWater_offset,
	SoftBodyWorldInfo_setWater_normal = CLIB.btSoftBodyWorldInfo_setWater_normal,
	DbvtNode_getVolume = CLIB.btDbvtNode_getVolume,
	SoftBodyWorldInfo_setWater_density = CLIB.btSoftBodyWorldInfo_setWater_density,
	OverlappingPairCallback_removeOverlappingPairsContainingProxy = CLIB.btOverlappingPairCallback_removeOverlappingPairsContainingProxy,
	JointFeedback_setAppliedForceBodyB = CLIB.btJointFeedback_setAppliedForceBodyB,
	SoftBody_appendLinearJoint4 = CLIB.btSoftBody_appendLinearJoint4,
	TranslationalLimitMotor_delete = CLIB.btTranslationalLimitMotor_delete,
	CollisionShape_calculateLocalInertia = CLIB.btCollisionShape_calculateLocalInertia,
	ConvexConvexAlgorithm_CreateFunc_new = CLIB.btConvexConvexAlgorithm_CreateFunc_new,
	SoftBodyWorldInfo_setDispatcher = CLIB.btSoftBodyWorldInfo_setDispatcher,
	MultiBodyDynamicsWorld_forwardKinematics = CLIB.btMultiBodyDynamicsWorld_forwardKinematics,
	OverlappingPairCache_findPair = CLIB.btOverlappingPairCache_findPair,
	SoftBodyWorldInfo_setBroadphase = CLIB.btSoftBodyWorldInfo_setBroadphase,
	SoftBodyWorldInfo_setAir_density = CLIB.btSoftBodyWorldInfo_setAir_density,
	TriangleIndexVertexMaterialArray_getLockedMaterialBase = CLIB.btTriangleIndexVertexMaterialArray_getLockedMaterialBase,
	SoftBody_Node_getIm = CLIB.btSoftBody_Node_getIm,
	OverlappingPairCallback_removeOverlappingPair = CLIB.btOverlappingPairCallback_removeOverlappingPair,
	MultiBodySolverConstraint_setUpperLimit = CLIB.btMultiBodySolverConstraint_setUpperLimit,
	DbvtBroadphase_setPrediction = CLIB.btDbvtBroadphase_setPrediction,
	SoftBodyWorldInfo_getMaxDisplacement = CLIB.btSoftBodyWorldInfo_getMaxDisplacement,
	StorageResult_setNormalOnSurfaceB = CLIB.btStorageResult_setNormalOnSurfaceB,
	SliderConstraint_calculateTransforms = CLIB.btSliderConstraint_calculateTransforms,
	SoftBodyWorldInfo_getGravity = CLIB.btSoftBodyWorldInfo_getGravity,
	SoftBodyWorldInfo_getDispatcher = CLIB.btSoftBodyWorldInfo_getDispatcher,
	CollisionAlgorithmConstructionInfo_setManifold = CLIB.btCollisionAlgorithmConstructionInfo_setManifold,
	SoftBodyWorldInfo_getAir_density = CLIB.btSoftBodyWorldInfo_getAir_density,
	SoftBodyHelpers_DrawFaceTree = CLIB.btSoftBodyHelpers_DrawFaceTree,
	SliderConstraint_testLinLimits = CLIB.btSliderConstraint_testLinLimits,
	SliderConstraint_testAngLimits = CLIB.btSliderConstraint_testAngLimits,
	Generic6DofSpringConstraint_enableSpring = CLIB.btGeneric6DofSpringConstraint_enableSpring,
	MultibodyLink_setAppliedConstraintForce = CLIB.btMultibodyLink_setAppliedConstraintForce,
	CompoundShapeChild_getChildShapeType = CLIB.btCompoundShapeChild_getChildShapeType,
	ConvexHullShape_addPoint = CLIB.btConvexHullShape_addPoint,
	SliderConstraint_setUpperLinLimit = CLIB.btSliderConstraint_setUpperLinLimit,
	SliderConstraint_setUpperAngLimit = CLIB.btSliderConstraint_setUpperAngLimit,
	SliderConstraint_setTargetLinMotorVelocity = CLIB.btSliderConstraint_setTargetLinMotorVelocity,
	TriangleInfoMap_new = CLIB.btTriangleInfoMap_new,
	PersistentManifold_setContactBreakingThreshold = CLIB.btPersistentManifold_setContactBreakingThreshold,
	DbvtAabbMm_delete = CLIB.btDbvtAabbMm_delete,
	ManifoldPoint_getPartId1 = CLIB.btManifoldPoint_getPartId1,
	SliderConstraint_getUseFrameOffset = CLIB.btSliderConstraint_getUseFrameOffset,
	SliderConstraint_setSoftnessLimAng = CLIB.btSliderConstraint_setSoftnessLimAng,
	SliderConstraint_setSoftnessDirAng = CLIB.btSliderConstraint_setSoftnessDirAng,
	SliderConstraint_getTargetAngMotorVelocity = CLIB.btSliderConstraint_getTargetAngMotorVelocity,
	ContactSolverInfoData_getRestitution = CLIB.btContactSolverInfoData_getRestitution,
	AlignedObjectArray_btSoftBody_Node_index_of = CLIB.btAlignedObjectArray_btSoftBody_Node_index_of,
	WorldImporter_getCollisionShapeByIndex = CLIB.btWorldImporter_getCollisionShapeByIndex,
	PersistentManifold_setCompanionIdB = CLIB.btPersistentManifold_setCompanionIdB,
	MultiBodySolverConstraint_setJacDiagABInv = CLIB.btMultiBodySolverConstraint_setJacDiagABInv,
	GImpactBvh_rayQuery = CLIB.btGImpactBvh_rayQuery,
	AlignedObjectArray_btVector3_at = CLIB.btAlignedObjectArray_btVector3_at,
	MultiBodySolverConstraint_getFriction = CLIB.btMultiBodySolverConstraint_getFriction,
	CompoundShape_getNumChildShapes = CLIB.btCompoundShape_getNumChildShapes,
	SliderConstraint_setPoweredLinMotor = CLIB.btSliderConstraint_setPoweredLinMotor,
	SoftBody_CJoint_getFriction = CLIB.btSoftBody_CJoint_getFriction,
	SliderConstraint_setMaxLinMotorForce = CLIB.btSliderConstraint_setMaxLinMotorForce,
	SliderConstraint_setMaxAngMotorForce = CLIB.btSliderConstraint_setMaxAngMotorForce,
	SliderConstraint_setLowerLinLimit = CLIB.btSliderConstraint_setLowerLinLimit,
	SliderConstraint_setLowerAngLimit = CLIB.btSliderConstraint_setLowerAngLimit,
	RotationalLimitMotor_getCurrentPosition = CLIB.btRotationalLimitMotor_getCurrentPosition,
	Box2dShape_getVertexCount = CLIB.btBox2dShape_getVertexCount,
	HingeConstraint_setLimit = CLIB.btHingeConstraint_setLimit,
	SliderConstraint_getRestitutionOrthoLin = CLIB.btSliderConstraint_getRestitutionOrthoLin,
	SliderConstraint_setDampingOrthoAng = CLIB.btSliderConstraint_setDampingOrthoAng,
	CollisionAlgorithmConstructionInfo_delete = CLIB.btCollisionAlgorithmConstructionInfo_delete,
	MultiBodyDynamicsWorld_addMultiBody3 = CLIB.btMultiBodyDynamicsWorld_addMultiBody3,
	SliderConstraint_setDampingLimLin = CLIB.btSliderConstraint_setDampingLimLin,
	BvhTree_isLeafNode = CLIB.btBvhTree_isLeafNode,
	SliderConstraint_setDampingLimAng = CLIB.btSliderConstraint_setDampingLimAng,
	AlignedObjectArray_btSoftBody_JointPtr_push_back = CLIB.btAlignedObjectArray_btSoftBody_JointPtr_push_back,
	DbvtBroadphase_collide = CLIB.btDbvtBroadphase_collide,
	WheelInfo_getEngineForce = CLIB.btWheelInfo_getEngineForce,
	SliderConstraint_setDampingDirAng = CLIB.btSliderConstraint_setDampingDirAng,
	SliderConstraint_setSoftnessOrthoAng = CLIB.btSliderConstraint_setSoftnessOrthoAng,
	ManifoldPoint_getLocalPointA = CLIB.btManifoldPoint_getLocalPointA,
	SliderConstraint_getUpperLinLimit = CLIB.btSliderConstraint_getUpperLinLimit,
	SubSimplexClosestResult_delete = CLIB.btSubSimplexClosestResult_delete,
	AxisSweep3_new2 = CLIB.btAxisSweep3_new2,
	Generic6DofSpring2Constraint_getRelativePivotPosition = CLIB.btGeneric6DofSpring2Constraint_getRelativePivotPosition,
	SliderConstraint_getTargetLinMotorVelocity = CLIB.btSliderConstraint_getTargetLinMotorVelocity,
	DbvtProxy_getStage = CLIB.btDbvtProxy_getStage,
	SliderConstraint_getSolveLinLimit = CLIB.btSliderConstraint_getSolveLinLimit,
	SliderConstraint_getSolveAngLimit = CLIB.btSliderConstraint_getSolveAngLimit,
	SliderConstraint_new2 = CLIB.btSliderConstraint_new2,
	SliderConstraint_getSoftnessLimLin = CLIB.btSliderConstraint_getSoftnessLimLin,
	RotationalLimitMotor_getStopERP = CLIB.btRotationalLimitMotor_getStopERP,
	RaycastVehicle_getForwardAxis = CLIB.btRaycastVehicle_getForwardAxis,
	SliderConstraint_setDampingOrthoLin = CLIB.btSliderConstraint_setDampingOrthoLin,
	SliderConstraint_getRestitutionOrthoAng = CLIB.btSliderConstraint_getRestitutionOrthoAng,
	CollisionObject_forceActivationState = CLIB.btCollisionObject_forceActivationState,
	MotionState_getWorldTransform = CLIB.btMotionState_getWorldTransform,
	RigidBody_getAngularDamping = CLIB.btRigidBody_getAngularDamping,
	MultiBodySolverConstraint_getDeltaVelBindex = CLIB.btMultiBodySolverConstraint_getDeltaVelBindex,
	PrimitiveTriangle_buildTriPlane = CLIB.btPrimitiveTriangle_buildTriPlane,
	SliderConstraint_getRestitutionDirAng = CLIB.btSliderConstraint_getRestitutionDirAng,
	SliderConstraint_getPoweredLinMotor = CLIB.btSliderConstraint_getPoweredLinMotor,
	SliderConstraint_getPoweredAngMotor = CLIB.btSliderConstraint_getPoweredAngMotor,
	RigidBody_saveKinematicState = CLIB.btRigidBody_saveKinematicState,
	SliderConstraint_getMaxLinMotorForce = CLIB.btSliderConstraint_getMaxLinMotorForce,
	SliderConstraint_getLowerLinLimit = CLIB.btSliderConstraint_getLowerLinLimit,
	SliderConstraint_getLowerAngLimit = CLIB.btSliderConstraint_getLowerAngLimit,
	SliderConstraint_getDampingDirAng = CLIB.btSliderConstraint_getDampingDirAng,
	SoftRigidDynamicsWorld_addSoftBody3 = CLIB.btSoftRigidDynamicsWorld_addSoftBody3,
	CollisionObject_checkCollideWithOverride = CLIB.btCollisionObject_checkCollideWithOverride,
	MultiBody_addBaseConstraintTorque = CLIB.btMultiBody_addBaseConstraintTorque,
	MotionState_delete = CLIB.btMotionState_delete,
	Generic6DofConstraint_getAngularLowerLimit = CLIB.btGeneric6DofConstraint_getAngularLowerLimit,
	AlignedObjectArray_btSoftBodyPtr_push_back = CLIB.btAlignedObjectArray_btSoftBodyPtr_push_back,
	SliderConstraint_getInfo1NonVirtual = CLIB.btSliderConstraint_getInfo1NonVirtual,
	UsageBitfield_getUsedVertexA = CLIB.btUsageBitfield_getUsedVertexA,
	SliderConstraint_getDampingOrthoLin = CLIB.btSliderConstraint_getDampingOrthoLin,
	SoftBody_pointersToIndices = CLIB.btSoftBody_pointersToIndices,
	SliderConstraint_getDampingDirLin = CLIB.btSliderConstraint_getDampingDirLin,
	RaycastVehicle_updateFriction = CLIB.btRaycastVehicle_updateFriction,
	SliderConstraint_getLinearPos = CLIB.btSliderConstraint_getLinearPos,
	SliderConstraint_getCalculatedTransformB = CLIB.btSliderConstraint_getCalculatedTransformB,
	SoftBody_Config_getTimescale = CLIB.btSoftBody_Config_getTimescale,
	ConvexPointCloudShape_setPoints3 = CLIB.btConvexPointCloudShape_setPoints3,
	SliderConstraint_getAncorInA = CLIB.btSliderConstraint_getAncorInA,
	SimulationIslandManager_delete = CLIB.btSimulationIslandManager_delete,
	HingeConstraint_getUpperLimit = CLIB.btHingeConstraint_getUpperLimit,
	SimulationIslandManager_setSplitIslands = CLIB.btSimulationIslandManager_setSplitIslands,
	Generic6DofConstraint_getLinearLowerLimit = CLIB.btGeneric6DofConstraint_getLinearLowerLimit,
	SimulationIslandManager_getSplitIslands = CLIB.btSimulationIslandManager_getSplitIslands,
	SimulationIslandManager_findUnions = CLIB.btSimulationIslandManager_findUnions,
	BoxBoxDetector_new = CLIB.btBoxBoxDetector_new,
	SimulationIslandManager_buildAndProcessIslands = CLIB.btSimulationIslandManager_buildAndProcessIslands,
	SimulationIslandManager_new = CLIB.btSimulationIslandManager_new,
	SimulationIslandManager_IslandCallback_processIsland = CLIB.btSimulationIslandManager_IslandCallback_processIsland,
	CollisionObjectWrapper_setIndex = CLIB.btCollisionObjectWrapper_setIndex,
	TriangleInfo_setEdgeV0V1Angle = CLIB.btTriangleInfo_setEdgeV0V1Angle,
	CollisionObject_getRollingFriction = CLIB.btCollisionObject_getRollingFriction,
	GImpactQuantizedBvh_getGlobalBox = CLIB.btGImpactQuantizedBvh_getGlobalBox,
	OptimizedBvhNode_delete = CLIB.btOptimizedBvhNode_delete,
	DbvtBroadphase_getStageRoots = CLIB.btDbvtBroadphase_getStageRoots,
	ShapeHull_numIndices = CLIB.btShapeHull_numIndices,
	DispatcherInfo_setEnableSatConvex = CLIB.btDispatcherInfo_setEnableSatConvex,
	ShapeHull_getVertexPointer = CLIB.btShapeHull_getVertexPointer,
	CollisionWorld_LocalShapeInfo_getShapePart = CLIB.btCollisionWorld_LocalShapeInfo_getShapePart,
	IndexedMesh_getVertexStride = CLIB.btIndexedMesh_getVertexStride,
	ShapeHull_buildHull = CLIB.btShapeHull_buildHull,
	ShapeHull_new = CLIB.btShapeHull_new,
	ConeTwistConstraint_getFrameOffsetB = CLIB.btConeTwistConstraint_getFrameOffsetB,
	QuantizedBvhTree_setNodeBound = CLIB.btQuantizedBvhTree_setNodeBound,
	DbvtBroadphase_new = CLIB.btDbvtBroadphase_new,
	DbvtNode_setData = CLIB.btDbvtNode_setData,
	DefaultSerializer_writeHeader = CLIB.btDefaultSerializer_writeHeader,
	Generic6DofSpringConstraint_isSpringEnabled = CLIB.btGeneric6DofSpringConstraint_isSpringEnabled,
	DefaultSerializer_new = CLIB.btDefaultSerializer_new,
	CollisionObject_hasContactResponse = CLIB.btCollisionObject_hasContactResponse,
	CollisionWorld_ClosestConvexResultCallback_getHitNormalWorld = CLIB.btCollisionWorld_ClosestConvexResultCallback_getHitNormalWorld,
	Chunk_setNumber = CLIB.btChunk_setNumber,
	PairSet_delete = CLIB.btPairSet_delete,
	WheelInfo_RaycastInfo_new = CLIB.btWheelInfo_RaycastInfo_new,
	ManifoldPoint_getContactPointFlags = CLIB.btManifoldPoint_getContactPointFlags,
	MultiBody_clearForcesAndTorques = CLIB.btMultiBody_clearForcesAndTorques,
	ManifoldResult_calculateCombinedRestitution = CLIB.btManifoldResult_calculateCombinedRestitution,
	WheelInfo_getDeltaRotation = CLIB.btWheelInfo_getDeltaRotation,
	AngularLimit_set2 = CLIB.btAngularLimit_set2,
	GImpactQuantizedBvh_rayQuery = CLIB.btGImpactQuantizedBvh_rayQuery,
	DbvtBroadphase_getVelocityPrediction = CLIB.btDbvtBroadphase_getVelocityPrediction,
	ConvexConcaveCollisionAlgorithm_new = CLIB.btConvexConcaveCollisionAlgorithm_new,
	Chunk_getDna_nr = CLIB.btChunk_getDna_nr,
	Chunk_getChunkCode = CLIB.btChunk_getChunkCode,
	SequentialImpulseConstraintSolver_setRandSeed = CLIB.btSequentialImpulseConstraintSolver_setRandSeed,
	MaterialProperties_getNumMaterials = CLIB.btMaterialProperties_getNumMaterials,
	RotationalLimitMotor_delete = CLIB.btRotationalLimitMotor_delete,
	SequentialImpulseConstraintSolver_btRand2 = CLIB.btSequentialImpulseConstraintSolver_btRand2,
	MultiBody_getWorldToBaseRot = CLIB.btMultiBody_getWorldToBaseRot,
	AlignedObjectArray_btSoftBodyPtr_at = CLIB.btAlignedObjectArray_btSoftBodyPtr_at,
	SequentialImpulseConstraintSolver_new = CLIB.btSequentialImpulseConstraintSolver_new,
	RigidBody_wantsSleeping = CLIB.btRigidBody_wantsSleeping,
	GImpactMeshShapePart_TrimeshPrimitiveManager_get_indices = CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_get_indices,
	Generic6DofSpring2Constraint_getRotationOrder = CLIB.btGeneric6DofSpring2Constraint_getRotationOrder,
	RigidBody_updateInertiaTensor = CLIB.btRigidBody_updateInertiaTensor,
	RigidBody_updateDeactivation = CLIB.btRigidBody_updateDeactivation,
	MultiSphereShape_getSphereCount = CLIB.btMultiSphereShape_getSphereCount,
	RigidBody_setSleepingThresholds = CLIB.btRigidBody_setSleepingThresholds,
	ManifoldPoint_setAppliedImpulse = CLIB.btManifoldPoint_setAppliedImpulse,
	PrimitiveManagerBase_delete = CLIB.btPrimitiveManagerBase_delete,
	MultiBodyConstraint_isUnilateral = CLIB.btMultiBodyConstraint_isUnilateral,
	TranslationalLimitMotor2_getStopCFM = CLIB.btTranslationalLimitMotor2_getStopCFM,
	ConvexConvexAlgorithm_setLowLevelOfDetail = CLIB.btConvexConvexAlgorithm_setLowLevelOfDetail,
	RigidBody_setMotionState = CLIB.btRigidBody_setMotionState,
	RigidBody_setLinearFactor = CLIB.btRigidBody_setLinearFactor,
	RigidBody_setInvInertiaDiagLocal = CLIB.btRigidBody_setInvInertiaDiagLocal,
	RigidBody_setFrictionSolverType = CLIB.btRigidBody_setFrictionSolverType,
	ConvexSeparatingDistanceUtil_new = CLIB.btConvexSeparatingDistanceUtil_new,
	CollisionDispatcher_setNearCallback = CLIB.btCollisionDispatcher_setNearCallback,
	BroadphaseAabbCallback_delete = CLIB.btBroadphaseAabbCallback_delete,
	ConvexCast_CastResult_setHitPoint = CLIB.btConvexCast_CastResult_setHitPoint,
	MultiBody_setBaseOmega = CLIB.btMultiBody_setBaseOmega,
	MultibodyLink_getAxes = CLIB.btMultibodyLink_getAxes,
	ManifoldPoint_setLocalPointA = CLIB.btManifoldPoint_setLocalPointA,
	RigidBody_setDamping = CLIB.btRigidBody_setDamping,
	RigidBody_setContactSolverType = CLIB.btRigidBody_setContactSolverType,
	SoftRigidDynamicsWorld_addSoftBody = CLIB.btSoftRigidDynamicsWorld_addSoftBody,
	RigidBody_setAngularVelocity = CLIB.btRigidBody_setAngularVelocity,
	SoftBody_applyForces = CLIB.btSoftBody_applyForces,
	Convex2dConvex2dAlgorithm_CreateFunc_getPdSolver = CLIB.btConvex2dConvex2dAlgorithm_CreateFunc_getPdSolver,
	ConvexPolyhedron_setExtents = CLIB.btConvexPolyhedron_setExtents,
	Convex2dConvex2dAlgorithm_CreateFunc_setSimplexSolver = CLIB.btConvex2dConvex2dAlgorithm_CreateFunc_setSimplexSolver,
	RigidBody_isInWorld = CLIB.btRigidBody_isInWorld,
	WheelInfo_setEngineForce = CLIB.btWheelInfo_setEngineForce,
	RigidBody_getTotalForce = CLIB.btRigidBody_getTotalForce,
	Dbvt_sStkNP_delete = CLIB.btDbvt_sStkNP_delete,
	CollisionWorld_LocalShapeInfo_delete = CLIB.btCollisionWorld_LocalShapeInfo_delete,
	CollisionWorld_LocalShapeInfo_setTriangleIndex = CLIB.btCollisionWorld_LocalShapeInfo_setTriangleIndex,
	SliderConstraint_setRestitutionOrthoAng = CLIB.btSliderConstraint_setRestitutionOrthoAng,
	DbvtBroadphase_getFixedleft = CLIB.btDbvtBroadphase_getFixedleft,
	RotationalLimitMotor_getAccumulatedImpulse = CLIB.btRotationalLimitMotor_getAccumulatedImpulse,
	BroadphaseRayCallback_getSigns = CLIB.btBroadphaseRayCallback_getSigns,
	QuantizedBvh_deSerializeDouble = CLIB.btQuantizedBvh_deSerializeDouble,
	SoftBody_clusterImpulse = CLIB.btSoftBody_clusterImpulse,
	RigidBody_getInvMass = CLIB.btRigidBody_getInvMass,
	GImpactQuantizedBvh_delete = CLIB.btGImpactQuantizedBvh_delete,
	MultiBodyConstraint_getMultiBodyB = CLIB.btMultiBodyConstraint_getMultiBodyB,
	RigidBody_getInvInertiaDiagLocal = CLIB.btRigidBody_getInvInertiaDiagLocal,
	MultiBodyDynamicsWorld_debugDrawMultiBodyConstraint = CLIB.btMultiBodyDynamicsWorld_debugDrawMultiBodyConstraint,
	ConvexHullShape_getUnscaledPoints = CLIB.btConvexHullShape_getUnscaledPoints,
	MultiBody_getJointTorque = CLIB.btMultiBody_getJointTorque,
	QuantizedBvhNode_getPartId = CLIB.btQuantizedBvhNode_getPartId,
	RigidBody_getConstraintRef = CLIB.btRigidBody_getConstraintRef,
	AlignedObjectArray_btCollisionObjectPtr_push_back = CLIB.btAlignedObjectArray_btCollisionObjectPtr_push_back,
	SoftBody_Cluster_getContainsAnchor = CLIB.btSoftBody_Cluster_getContainsAnchor,
	RigidBody_getBroadphaseProxy = CLIB.btRigidBody_getBroadphaseProxy,
	RigidBody_getAngularSleepingThreshold = CLIB.btRigidBody_getAngularSleepingThreshold,
	ConeShape_new = CLIB.btConeShape_new,
	BvhTriangleMeshShape_performConvexcast = CLIB.btBvhTriangleMeshShape_performConvexcast,
	ConcaveShape_processAllTriangles = CLIB.btConcaveShape_processAllTriangles,
	CompoundShapeChild_delete = CLIB.btCompoundShapeChild_delete,
	ScaledBvhTriangleMeshShape_getChildShape = CLIB.btScaledBvhTriangleMeshShape_getChildShape,
	RigidBody_applyTorque = CLIB.btRigidBody_applyTorque,
	GImpactMeshShapePart_TrimeshPrimitiveManager_getNumfaces = CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_getNumfaces,
	DbvtBroadphase_setCid = CLIB.btDbvtBroadphase_setCid,
	HingeConstraint_getInfo2NonVirtual = CLIB.btHingeConstraint_getInfo2NonVirtual,
	ConvexSeparatingDistanceUtil_delete = CLIB.btConvexSeparatingDistanceUtil_delete,
	DbvtBroadphase_setNeedcleanup = CLIB.btDbvtBroadphase_setNeedcleanup,
	SoftBodySolver_copyBackToSoftBodies = CLIB.btSoftBodySolver_copyBackToSoftBodies,
	MultiBody_setBaseMass = CLIB.btMultiBody_setBaseMass,
	RigidBody_applyDamping = CLIB.btRigidBody_applyDamping,
	RigidBody_applyCentralImpulse = CLIB.btRigidBody_applyCentralImpulse,
	RigidBody_addConstraintRef = CLIB.btRigidBody_addConstraintRef,
	RigidBody_btRigidBodyConstructionInfo_setStartWorldTransform = CLIB.btRigidBody_btRigidBodyConstructionInfo_setStartWorldTransform,
	HingeConstraint_getLimitRelaxationFactor = CLIB.btHingeConstraint_getLimitRelaxationFactor,
	RigidBody_btRigidBodyConstructionInfo_setRollingFriction = CLIB.btRigidBody_btRigidBodyConstructionInfo_setRollingFriction,
	RigidBody_btRigidBodyConstructionInfo_setRestitution = CLIB.btRigidBody_btRigidBodyConstructionInfo_setRestitution,
	Generic6DofConstraint_setLimit = CLIB.btGeneric6DofConstraint_setLimit,
	RigidBody_btRigidBodyConstructionInfo_setMass = CLIB.btRigidBody_btRigidBodyConstructionInfo_setMass,
	WheelInfo_setRollInfluence = CLIB.btWheelInfo_setRollInfluence,
	RigidBody_btRigidBodyConstructionInfo_setLinearSleepingThreshold = CLIB.btRigidBody_btRigidBodyConstructionInfo_setLinearSleepingThreshold,
	MultiBodyDynamicsWorld_clearMultiBodyConstraintForces = CLIB.btMultiBodyDynamicsWorld_clearMultiBodyConstraintForces,
	DbvtAabbMm_Contain = CLIB.btDbvtAabbMm_Contain,
	RigidBody_btRigidBodyConstructionInfo_setLinearDamping = CLIB.btRigidBody_btRigidBodyConstructionInfo_setLinearDamping,
	Dbvt_clone2 = CLIB.btDbvt_clone2,
	RigidBody_btRigidBodyConstructionInfo_setFriction = CLIB.btRigidBody_btRigidBodyConstructionInfo_setFriction,
	RigidBody_btRigidBodyConstructionInfo_setCollisionShape = CLIB.btRigidBody_btRigidBodyConstructionInfo_setCollisionShape,
	PrimitiveTriangle_find_triangle_collision_clip_method = CLIB.btPrimitiveTriangle_find_triangle_collision_clip_method,
	RigidBody_btRigidBodyConstructionInfo_setAngularDamping = CLIB.btRigidBody_btRigidBodyConstructionInfo_setAngularDamping,
	CollisionWorld_computeOverlappingPairs = CLIB.btCollisionWorld_computeOverlappingPairs,
	CollisionDispatcher_setDispatcherFlags = CLIB.btCollisionDispatcher_setDispatcherFlags,
	RigidBody_btRigidBodyConstructionInfo_setAdditionalDampingFactor = CLIB.btRigidBody_btRigidBodyConstructionInfo_setAdditionalDampingFactor,
	RotationalLimitMotor_new = CLIB.btRotationalLimitMotor_new,
	RigidBody_btRigidBodyConstructionInfo_setAdditionalDamping = CLIB.btRigidBody_btRigidBodyConstructionInfo_setAdditionalDamping,
	RigidBody_btRigidBodyConstructionInfo_setAdditionalAngularDampingThresholdSqr = CLIB.btRigidBody_btRigidBodyConstructionInfo_setAdditionalAngularDampingThresholdSqr,
	MultiBody_hasSelfCollision = CLIB.btMultiBody_hasSelfCollision,
	ManifoldResult_new2 = CLIB.btManifoldResult_new2,
	PolarDecomposition_maxIterations = CLIB.btPolarDecomposition_maxIterations,
	RigidBody_btRigidBodyConstructionInfo_setAdditionalAngularDampingFactor = CLIB.btRigidBody_btRigidBodyConstructionInfo_setAdditionalAngularDampingFactor,
	RigidBody_btRigidBodyConstructionInfo_getStartWorldTransform = CLIB.btRigidBody_btRigidBodyConstructionInfo_getStartWorldTransform,
	TranslationalLimitMotor_getMaxMotorForce = CLIB.btTranslationalLimitMotor_getMaxMotorForce,
	SparseSdf3_Initialize3 = CLIB.btSparseSdf3_Initialize3,
	DefaultCollisionConstructionInfo_setCollisionAlgorithmPool = CLIB.btDefaultCollisionConstructionInfo_setCollisionAlgorithmPool,
	RigidBody_btRigidBodyConstructionInfo_getMass = CLIB.btRigidBody_btRigidBodyConstructionInfo_getMass,
	Box2dShape_getVertices = CLIB.btBox2dShape_getVertices,
	RigidBody_btRigidBodyConstructionInfo_getLocalInertia = CLIB.btRigidBody_btRigidBodyConstructionInfo_getLocalInertia,
	CollisionAlgorithmConstructionInfo_getManifold = CLIB.btCollisionAlgorithmConstructionInfo_getManifold,
	RigidBody_btRigidBodyConstructionInfo_getLinearDamping = CLIB.btRigidBody_btRigidBodyConstructionInfo_getLinearDamping,
	Dbvt_setLkhd = CLIB.btDbvt_setLkhd,
	RigidBody_btRigidBodyConstructionInfo_getCollisionShape = CLIB.btRigidBody_btRigidBodyConstructionInfo_getCollisionShape,
	ConvexCast_CastResult_getHitTransformB = CLIB.btConvexCast_CastResult_getHitTransformB,
	Dbvt_sStkNN_setB = CLIB.btDbvt_sStkNN_setB,
	RigidBody_btRigidBodyConstructionInfo_getAngularDamping = CLIB.btRigidBody_btRigidBodyConstructionInfo_getAngularDamping,
	GjkPairDetector_setMinkowskiB = CLIB.btGjkPairDetector_setMinkowskiB,
	QuantizedBvhTree_getNodeCount = CLIB.btQuantizedBvhTree_getNodeCount,
	RigidBody_btRigidBodyConstructionInfo_getAdditionalDampingFactor = CLIB.btRigidBody_btRigidBodyConstructionInfo_getAdditionalDampingFactor,
	MultiBody_addLinkConstraintTorque = CLIB.btMultiBody_addLinkConstraintTorque,
	IndexedMesh_setVertexType = CLIB.btIndexedMesh_setVertexType,
	AlignedObjectArray_btPersistentManifoldPtr_push_back = CLIB.btAlignedObjectArray_btPersistentManifoldPtr_push_back,
	RaycastVehicle_updateWheelTransform2 = CLIB.btRaycastVehicle_updateWheelTransform2,
	RaycastVehicle_updateWheelTransform = CLIB.btRaycastVehicle_updateWheelTransform,
	RaycastVehicle_updateVehicle = CLIB.btRaycastVehicle_updateVehicle,
	RaycastVehicle_updateSuspension = CLIB.btRaycastVehicle_updateSuspension,
	SoftBody_updateArea = CLIB.btSoftBody_updateArea,
	ActionInterface_updateAction = CLIB.btActionInterface_updateAction,
	ConvexCast_CastResult_setHitTransformB = CLIB.btConvexCast_CastResult_setHitTransformB,
	ManifoldPoint_setLifeTime = CLIB.btManifoldPoint_setLifeTime,
	KinematicCharacterController_setUseGhostSweepTest = CLIB.btKinematicCharacterController_setUseGhostSweepTest,
	MultibodyLink_setDofOffset = CLIB.btMultibodyLink_setDofOffset,
	GImpactShapeInterface_processAllTrianglesRay = CLIB.btGImpactShapeInterface_processAllTrianglesRay,
	StorageResult_getNormalOnSurfaceB = CLIB.btStorageResult_getNormalOnSurfaceB,
	SoftBody_appendLinearJoint = CLIB.btSoftBody_appendLinearJoint,
	CollisionWorld_new = CLIB.btCollisionWorld_new,
	CollisionObjectWrapper_getCollisionShape = CLIB.btCollisionObjectWrapper_getCollisionShape,
	CompoundShape_removeChildShapeByIndex = CLIB.btCompoundShape_removeChildShapeByIndex,
	PersistentManifold_validContactDistance = CLIB.btPersistentManifold_validContactDistance,
	RaycastVehicle_getWheelInfo2 = CLIB.btRaycastVehicle_getWheelInfo2,
	RaycastVehicle_getWheelInfo = CLIB.btRaycastVehicle_getWheelInfo,
	BroadphaseProxy_isConvex = CLIB.btBroadphaseProxy_isConvex,
	RaycastVehicle_getUserConstraintType = CLIB.btRaycastVehicle_getUserConstraintType,
	Point2PointConstraint_updateRHS = CLIB.btPoint2PointConstraint_updateRHS,
	Generic6DofSpringConstraint_getDamping = CLIB.btGeneric6DofSpringConstraint_getDamping,
	RaycastVehicle_getRightAxis = CLIB.btRaycastVehicle_getRightAxis,
	PersistentManifold_getNumContacts = CLIB.btPersistentManifold_getNumContacts,
	DiscreteCollisionDetectorInterface_ClosestPointInput_new = CLIB.btDiscreteCollisionDetectorInterface_ClosestPointInput_new,
	RaycastVehicle_getNumWheels = CLIB.btRaycastVehicle_getNumWheels,
	GearConstraint_setRatio = CLIB.btGearConstraint_setRatio,
	MultibodyLink_updateCacheMultiDof = CLIB.btMultibodyLink_updateCacheMultiDof,
	RaycastVehicle_new = CLIB.btRaycastVehicle_new,
	RaycastVehicle_btVehicleTuning_setSuspensionStiffness = CLIB.btRaycastVehicle_btVehicleTuning_setSuspensionStiffness,
	TriangleInfoMap_getZeroAreaThreshold = CLIB.btTriangleInfoMap_getZeroAreaThreshold,
	RaycastVehicle_btVehicleTuning_setSuspensionCompression = CLIB.btRaycastVehicle_btVehicleTuning_setSuspensionCompression,
	VoronoiSimplexSolver_getSimplexVectorW = CLIB.btVoronoiSimplexSolver_getSimplexVectorW,
	RaycastVehicle_btVehicleTuning_setFrictionSlip = CLIB.btRaycastVehicle_btVehicleTuning_setFrictionSlip,
	RigidBody_btRigidBodyConstructionInfo_getLinearSleepingThreshold = CLIB.btRigidBody_btRigidBodyConstructionInfo_getLinearSleepingThreshold,
	RaycastVehicle_btVehicleTuning_getMaxSuspensionTravelCm = CLIB.btRaycastVehicle_btVehicleTuning_getMaxSuspensionTravelCm,
	DefaultMotionState_new3 = CLIB.btDefaultMotionState_new3,
	RaycastVehicle_btVehicleTuning_getMaxSuspensionForce = CLIB.btRaycastVehicle_btVehicleTuning_getMaxSuspensionForce,
	QuantizedBvh_unQuantize = CLIB.btQuantizedBvh_unQuantize,
	QuantizedBvh_setTraversalMode = CLIB.btQuantizedBvh_setTraversalMode,
	GImpactCollisionAlgorithm_getFace0 = CLIB.btGImpactCollisionAlgorithm_getFace0,
	QuantizedBvh_serialize2 = CLIB.btQuantizedBvh_serialize2,
	QuantizedBvh_serialize = CLIB.btQuantizedBvh_serialize,
	QuantizedBvh_reportBoxCastOverlappingNodex = CLIB.btQuantizedBvh_reportBoxCastOverlappingNodex,
	QuantizedBvh_reportAabbOverlappingNodex = CLIB.btQuantizedBvh_reportAabbOverlappingNodex,
	ConeShape_getRadius = CLIB.btConeShape_getRadius,
	AABB_new2 = CLIB.btAABB_new2,
	AngularLimit_getSoftness = CLIB.btAngularLimit_getSoftness,
	TriangleIndexVertexMaterialArray_getLockedReadOnlyMaterialBase = CLIB.btTriangleIndexVertexMaterialArray_getLockedReadOnlyMaterialBase,
	AxisSweep3_new3 = CLIB.btAxisSweep3_new3,
	QuantizedBvh_getQuantizedNodeArray = CLIB.btQuantizedBvh_getQuantizedNodeArray,
	GImpactBvh_hasHierarchy = CLIB.btGImpactBvh_hasHierarchy,
	MultiBody_computeAccelerationsArticulatedBodyAlgorithmMultiDof2 = CLIB.btMultiBody_computeAccelerationsArticulatedBodyAlgorithmMultiDof2,
	UnionFind_isRoot = CLIB.btUnionFind_isRoot,
	QuantizedBvh_deSerializeInPlace = CLIB.btQuantizedBvh_deSerializeInPlace,
	WorldImporter_createGimpactShape = CLIB.btWorldImporter_createGimpactShape,
	MultiBodySolverConstraint_getRhs = CLIB.btMultiBodySolverConstraint_getRhs,
	NodeOverlapCallback_delete = CLIB.btNodeOverlapCallback_delete,
	MultibodyLink_setAxisBottom = CLIB.btMultibodyLink_setAxisBottom,
	ManifoldPoint_setLocalPointB = CLIB.btManifoldPoint_setLocalPointB,
	DynamicsWorld_synchronizeMotionStates = CLIB.btDynamicsWorld_synchronizeMotionStates,
	MultiBody_setJointVelMultiDof = CLIB.btMultiBody_setJointVelMultiDof,
	SoftBody_Link_getC3 = CLIB.btSoftBody_Link_getC3,
	CollisionWorld_RayResultCallback_setClosestHitFraction = CLIB.btCollisionWorld_RayResultCallback_setClosestHitFraction,
	ShapeHull_numVertices = CLIB.btShapeHull_numVertices,
	OptimizedBvhNode_setTriangleIndex = CLIB.btOptimizedBvhNode_setTriangleIndex,
	QuantizedBvhTree_getNodeData = CLIB.btQuantizedBvhTree_getNodeData,
	SoftBody_getTetraVertexData = CLIB.btSoftBody_getTetraVertexData,
	OptimizedBvhNode_setEscapeIndex = CLIB.btOptimizedBvhNode_setEscapeIndex,
	OptimizedBvhNode_setAabbMinOrg = CLIB.btOptimizedBvhNode_setAabbMinOrg,
	MultibodyLink_getJointPos = CLIB.btMultibodyLink_getJointPos,
	OptimizedBvhNode_setAabbMaxOrg = CLIB.btOptimizedBvhNode_setAabbMaxOrg,
	MultiBodyConstraint_getAppliedImpulse = CLIB.btMultiBodyConstraint_getAppliedImpulse,
	OptimizedBvhNode_getEscapeIndex = CLIB.btOptimizedBvhNode_getEscapeIndex,
	DiscreteDynamicsWorld_applyGravity = CLIB.btDiscreteDynamicsWorld_applyGravity,
	Dbvt_clear = CLIB.btDbvt_clear,
	BroadphasePair_new3 = CLIB.btBroadphasePair_new3,
	OptimizedBvhNode_getAabbMinOrg = CLIB.btOptimizedBvhNode_getAabbMinOrg,
	AlignedObjectArray_btIndexedMesh_size = CLIB.btAlignedObjectArray_btIndexedMesh_size,
	HingeConstraint_new = CLIB.btHingeConstraint_new,
	TranslationalLimitMotor_setStopERP = CLIB.btTranslationalLimitMotor_setStopERP,
	OptimizedBvhNode_new = CLIB.btOptimizedBvhNode_new,
	QuantizedBvhNode_delete = CLIB.btQuantizedBvhNode_delete,
	QuantizedBvhNode_setEscapeIndexOrTriangleIndex = CLIB.btQuantizedBvhNode_setEscapeIndexOrTriangleIndex,
	ConvexInternalShape_getLocalScalingNV = CLIB.btConvexInternalShape_getLocalScalingNV,
	QuantizedBvhNode_isLeafNode = CLIB.btQuantizedBvhNode_isLeafNode,
	Generic6DofSpring2Constraint_setAngularUpperLimit = CLIB.btGeneric6DofSpring2Constraint_setAngularUpperLimit,
	QuantizedBvhNode_getTriangleIndex = CLIB.btQuantizedBvhNode_getTriangleIndex,
	QuantizedBvhNode_getQuantizedAabbMin = CLIB.btQuantizedBvhNode_getQuantizedAabbMin,
	BroadphaseInterface_rayTest = CLIB.btBroadphaseInterface_rayTest,
	BoxBoxCollisionAlgorithm_CreateFunc_new = CLIB.btBoxBoxCollisionAlgorithm_CreateFunc_new,
	RigidBody_getContactSolverType = CLIB.btRigidBody_getContactSolverType,
	JointFeedback_getAppliedTorqueBodyA = CLIB.btJointFeedback_getAppliedTorqueBodyA,
	QuantizedBvhNode_new = CLIB.btQuantizedBvhNode_new,
	PolyhedralConvexAabbCachingShape_getNonvirtualAabb = CLIB.btPolyhedralConvexAabbCachingShape_getNonvirtualAabb,
	PolyhedralConvexShape_isInside = CLIB.btPolyhedralConvexShape_isInside,
	RotationalLimitMotor2_getMotorCFM = CLIB.btRotationalLimitMotor2_getMotorCFM,
	OptimizedBvh_refit = CLIB.btOptimizedBvh_refit,
	MultiBody_calculateSerializeBufferSize = CLIB.btMultiBody_calculateSerializeBufferSize,
	NullPairCache_new = CLIB.btNullPairCache_new,
	TypedConstraint_getDbgDrawSize = CLIB.btTypedConstraint_getDbgDrawSize,
	CharacterControllerInterface_jump = CLIB.btCharacterControllerInterface_jump,
	MultiBodySolverConstraint_getCfm = CLIB.btMultiBodySolverConstraint_getCfm,
	PolyhedralConvexShape_getNumVertices = CLIB.btPolyhedralConvexShape_getNumVertices,
	SubSimplexClosestResult_reset = CLIB.btSubSimplexClosestResult_reset,
	GhostObject_addOverlappingObjectInternal = CLIB.btGhostObject_addOverlappingObjectInternal,
	PolyhedralConvexShape_getEdge = CLIB.btPolyhedralConvexShape_getEdge,
	AlignedObjectArray_btSoftBody_Anchor_at = CLIB.btAlignedObjectArray_btSoftBody_Anchor_at,
	MaterialProperties_setTriangleMaterialStride = CLIB.btMaterialProperties_setTriangleMaterialStride,
	MultiBody_computeAccelerationsArticulatedBodyAlgorithmMultiDof = CLIB.btMultiBody_computeAccelerationsArticulatedBodyAlgorithmMultiDof,
	Generic6DofSpring2Constraint_setDamping = CLIB.btGeneric6DofSpring2Constraint_setDamping,
	PolarDecomposition_decompose = CLIB.btPolarDecomposition_decompose,
	GImpactQuantizedBvh_getNodeTriangle = CLIB.btGImpactQuantizedBvh_getNodeTriangle,
	PolarDecomposition_new3 = CLIB.btPolarDecomposition_new3,
	DbvtBroadphase_getPid = CLIB.btDbvtBroadphase_getPid,
	Generic6DofSpring2Constraint_getAngularLowerLimitReversed = CLIB.btGeneric6DofSpring2Constraint_getAngularLowerLimitReversed,
	Box2dBox2dCollisionAlgorithm_new2 = CLIB.btBox2dBox2dCollisionAlgorithm_new2,
	CollisionWorld_objectQuerySingleInternal = CLIB.btCollisionWorld_objectQuerySingleInternal,
	PointCollector_setHasResult = CLIB.btPointCollector_setHasResult,
	PointCollector_setDistance = CLIB.btPointCollector_setDistance,
	MultiSphereShape_new2 = CLIB.btMultiSphereShape_new2,
	TranslationalLimitMotor_getCurrentLimitError = CLIB.btTranslationalLimitMotor_getCurrentLimitError,
	PointCollector_getNormalOnBInWorld = CLIB.btPointCollector_getNormalOnBInWorld,
	PointCollector_getHasResult = CLIB.btPointCollector_getHasResult,
	PointCollector_getDistance = CLIB.btPointCollector_getDistance,
	PointCollector_new = CLIB.btPointCollector_new,
	Generic6DofSpring2Constraint_getFrameOffsetA = CLIB.btGeneric6DofSpring2Constraint_getFrameOffsetA,
	BvhTree_new = CLIB.btBvhTree_new,
	RaycastVehicle_getUserConstraintId = CLIB.btRaycastVehicle_getUserConstraintId,
	ConvexPolyhedron_getFaces = CLIB.btConvexPolyhedron_getFaces,
	RigidBody_getNumConstraintRefs = CLIB.btRigidBody_getNumConstraintRefs,
	MultiBodyDynamicsWorld_addMultiBody = CLIB.btMultiBodyDynamicsWorld_addMultiBody,
	Point2PointConstraint_getPivotInB = CLIB.btPoint2PointConstraint_getPivotInB,
	GImpactShapeInterface_getPrimitiveManager = CLIB.btGImpactShapeInterface_getPrimitiveManager,
	SoftBody_RayFromToCaster_getRayFrom = CLIB.btSoftBody_RayFromToCaster_getRayFrom,
	QuantizedBvhTree_build_tree = CLIB.btQuantizedBvhTree_build_tree,
	OverlapCallback_delete = CLIB.btOverlapCallback_delete,
	Point2PointConstraint_getInfo2NonVirtual = CLIB.btPoint2PointConstraint_getInfo2NonVirtual,
	CollisionWorld_RayResultCallback_delete = CLIB.btCollisionWorld_RayResultCallback_delete,
	Point2PointConstraint_getInfo1NonVirtual = CLIB.btPoint2PointConstraint_getInfo1NonVirtual,
	ConeTwistConstraint_enableMotor = CLIB.btConeTwistConstraint_enableMotor,
	Point2PointConstraint_new = CLIB.btPoint2PointConstraint_new,
	Dbvt_ICollide_Descent = CLIB.btDbvt_ICollide_Descent,
	ConstraintSetting_setTau = CLIB.btConstraintSetting_setTau,
	SoftBody_appendLink = CLIB.btSoftBody_appendLink,
	ConstraintSetting_setDamping = CLIB.btConstraintSetting_setDamping,
	Chunk_setChunkCode = CLIB.btChunk_setChunkCode,
	SliderConstraint_getRestitutionLimAng = CLIB.btSliderConstraint_getRestitutionLimAng,
	MultiBody_isAwake = CLIB.btMultiBody_isAwake,
	ConstraintSetting_new = CLIB.btConstraintSetting_new,
	GImpactCollisionAlgorithm_CreateFunc_new = CLIB.btGImpactCollisionAlgorithm_CreateFunc_new,
	MultiBody_finalizeMultiDof = CLIB.btMultiBody_finalizeMultiDof,
	MultiBodyDynamicsWorld_getMultiBody = CLIB.btMultiBodyDynamicsWorld_getMultiBody,
	DbvtAabbMm_Classify = CLIB.btDbvtAabbMm_Classify,
	MultiBody_setupRevolute = CLIB.btMultiBody_setupRevolute,
	HingeConstraint_getAngularOnly = CLIB.btHingeConstraint_getAngularOnly,
	GImpactMeshShapePart_TrimeshPrimitiveManager_setMeshInterface = CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_setMeshInterface,
	ConeTwistConstraint_getFrameOffsetA = CLIB.btConeTwistConstraint_getFrameOffsetA,
	PersistentManifold_setNumContacts = CLIB.btPersistentManifold_setNumContacts,
	WorldImporter_setDynamicsWorldInfo = CLIB.btWorldImporter_setDynamicsWorldInfo,
	PersistentManifold_setContactProcessingThreshold = CLIB.btPersistentManifold_setContactProcessingThreshold,
	SliderConstraint_setSoftnessOrthoLin = CLIB.btSliderConstraint_setSoftnessOrthoLin,
	CollisionWorld_getCollisionObjectArray = CLIB.btCollisionWorld_getCollisionObjectArray,
	RigidBody_getMotionState = CLIB.btRigidBody_getMotionState,
	PersistentManifold_setCompanionIdA = CLIB.btPersistentManifold_setCompanionIdA,
	DynamicsWorld_setConstraintSolver = CLIB.btDynamicsWorld_setConstraintSolver,
	SoftBody_updateConstants = CLIB.btSoftBody_updateConstants,
	TranslationalLimitMotor_setNormalCFM = CLIB.btTranslationalLimitMotor_setNormalCFM,
	SoftBody_Anchor_getBody = CLIB.btSoftBody_Anchor_getBody,
	PersistentManifold_getContactPoint = CLIB.btPersistentManifold_getContactPoint,
	CollisionAlgorithm_processCollision = CLIB.btCollisionAlgorithm_processCollision,
	SoftBody_getCfg = CLIB.btSoftBody_getCfg,
	PersistentManifold_getBody0 = CLIB.btPersistentManifold_getBody0,
	PersistentManifold_clearUserCache = CLIB.btPersistentManifold_clearUserCache,
	PersistentManifold_clearManifold = CLIB.btPersistentManifold_clearManifold,
	RotationalLimitMotor2_getSpringDamping = CLIB.btRotationalLimitMotor2_getSpringDamping,
	PersistentManifold_new = CLIB.btPersistentManifold_new,
	OverlappingPairCallback_addOverlappingPair = CLIB.btOverlappingPairCallback_addOverlappingPair,
	ConvexCast_CastResult_reportFailure = CLIB.btConvexCast_CastResult_reportFailure,
	GImpactCollisionAlgorithm_getPart1 = CLIB.btGImpactCollisionAlgorithm_getPart1,
	DispatcherInfo_setEnableSPU = CLIB.btDispatcherInfo_setEnableSPU,
	MultibodyLink_getJointTorque = CLIB.btMultibodyLink_getJointTorque,
	HingeAccumulatedAngleConstraint_setAccumulatedHingeAngle = CLIB.btHingeAccumulatedAngleConstraint_setAccumulatedHingeAngle,
	Generic6DofSpring2Constraint_setMaxMotorForce = CLIB.btGeneric6DofSpring2Constraint_setMaxMotorForce,
	SortedOverlappingPairCache_new = CLIB.btSortedOverlappingPairCache_new,
	HashedOverlappingPairCache_getOverlapFilterCallback = CLIB.btHashedOverlappingPairCache_getOverlapFilterCallback,
	HashedOverlappingPairCache_GetCount = CLIB.btHashedOverlappingPairCache_GetCount,
	Box2dShape_getHalfExtentsWithoutMargin = CLIB.btBox2dShape_getHalfExtentsWithoutMargin,
	HashedOverlappingPairCache_new = CLIB.btHashedOverlappingPairCache_new,
	OverlappingPairCache_setOverlapFilterCallback = CLIB.btOverlappingPairCache_setOverlapFilterCallback,
	OverlappingPairCache_processAllOverlappingPairs = CLIB.btOverlappingPairCache_processAllOverlappingPairs,
	HingeConstraint_getInfo1NonVirtual = CLIB.btHingeConstraint_getInfo1NonVirtual,
	Generic6DofSpring2Constraint_new2 = CLIB.btGeneric6DofSpring2Constraint_new2,
	OverlappingPairCache_getNumOverlappingPairs = CLIB.btOverlappingPairCache_getNumOverlappingPairs,
	OverlappingPairCache_cleanOverlappingPair = CLIB.btOverlappingPairCache_cleanOverlappingPair,
	CollisionShape_calculateSerializeBufferSize = CLIB.btCollisionShape_calculateSerializeBufferSize,
	GImpactMeshShapePart_new = CLIB.btGImpactMeshShapePart_new,
	ConvexShape_project = CLIB.btConvexShape_project,
	HingeConstraint_getLimitSoftness = CLIB.btHingeConstraint_getLimitSoftness,
	OptimizedBvh_serializeInPlace = CLIB.btOptimizedBvh_serializeInPlace,
	RotationalLimitMotor2_getSpringStiffness = CLIB.btRotationalLimitMotor2_getSpringStiffness,
	PolyhedralConvexShape_initializePolyhedralFeatures = CLIB.btPolyhedralConvexShape_initializePolyhedralFeatures,
	DbvtAabbMm_tMaxs = CLIB.btDbvtAabbMm_tMaxs,
	HingeConstraint_new3 = CLIB.btHingeConstraint_new3,
	AxisSweep3_getHandle = CLIB.btAxisSweep3_getHandle,
	BroadphaseInterface_getAabb = CLIB.btBroadphaseInterface_getAabb,
	OptimizedBvh_deSerializeInPlace = CLIB.btOptimizedBvh_deSerializeInPlace,
	JointFeedback_setAppliedTorqueBodyB = CLIB.btJointFeedback_setAppliedTorqueBodyB,
	GeometryUtil_areVerticesBehindPlane = CLIB.btGeometryUtil_areVerticesBehindPlane,
	NNCGConstraintSolver_new = CLIB.btNNCGConstraintSolver_new,
	HingeConstraint_setFrames = CLIB.btHingeConstraint_setFrames,
	MultiSphereShape_getSpherePosition = CLIB.btMultiSphereShape_getSpherePosition,
	RigidBody_upcast = CLIB.btRigidBody_upcast,
	RotationalLimitMotor2_new2 = CLIB.btRotationalLimitMotor2_new2,
	SoftBodyHelpers_CreateFromConvexHull2 = CLIB.btSoftBodyHelpers_CreateFromConvexHull2,
	TypedConstraint_getOverrideNumSolverIterations = CLIB.btTypedConstraint_getOverrideNumSolverIterations,
	MultimaterialTriangleMeshShape_new4 = CLIB.btMultimaterialTriangleMeshShape_new4,
	MultimaterialTriangleMeshShape_new = CLIB.btMultimaterialTriangleMeshShape_new,
	WorldImporter_getTriangleInfoMapByIndex = CLIB.btWorldImporter_getTriangleInfoMapByIndex,
	SoftBodyWorldInfo_getSparsesdf = CLIB.btSoftBodyWorldInfo_getSparsesdf,
	CollisionWorld_AllHitsRayResultCallback_getRayToWorld = CLIB.btCollisionWorld_AllHitsRayResultCallback_getRayToWorld,
	ConvexPointCloudShape_new3 = CLIB.btConvexPointCloudShape_new3,
	MultiBodySolverConstraint_setUnusedPadding4 = CLIB.btMultiBodySolverConstraint_setUnusedPadding4,
	MultiBodySolverConstraint_setSolverBodyIdB = CLIB.btMultiBodySolverConstraint_setSolverBodyIdB,
	WheelInfo_RaycastInfo_getWheelAxleWS = CLIB.btWheelInfo_RaycastInfo_getWheelAxleWS,
	MultiBodySolverConstraint_setRhsPenetration = CLIB.btMultiBodySolverConstraint_setRhsPenetration,
	MultiBodySolverConstraint_setRhs = CLIB.btMultiBodySolverConstraint_setRhs,
	UsageBitfield_setUsedVertexA = CLIB.btUsageBitfield_setUsedVertexA,
	ContactSolverInfoData_setSplitImpulseTurnErp = CLIB.btContactSolverInfoData_setSplitImpulseTurnErp,
	WheelInfo_setWheelAxleCS = CLIB.btWheelInfo_setWheelAxleCS,
	MultiBodySolverConstraint_setOverrideNumSolverIterations = CLIB.btMultiBodySolverConstraint_setOverrideNumSolverIterations,
	MultiBodySolverConstraint_setOriginalContactPoint = CLIB.btMultiBodySolverConstraint_setOriginalContactPoint,
	ManifoldResult_new = CLIB.btManifoldResult_new,
	MultiBodySolverConstraint_setOrgDofIndex = CLIB.btMultiBodySolverConstraint_setOrgDofIndex,
	CollisionWorld_LocalRayResult_getLocalShapeInfo = CLIB.btCollisionWorld_LocalRayResult_getLocalShapeInfo,
	AlignedObjectArray_btBroadphasePair_resizeNoInitialize = CLIB.btAlignedObjectArray_btBroadphasePair_resizeNoInitialize,
	MultiBodySolverConstraint_setOrgConstraint = CLIB.btMultiBodySolverConstraint_setOrgConstraint,
	GImpactCollisionAlgorithm_gimpact_vs_gimpact = CLIB.btGImpactCollisionAlgorithm_gimpact_vs_gimpact,
	MultiBodySolverConstraint_setMultiBodyB = CLIB.btMultiBodySolverConstraint_setMultiBodyB,
	Dbvt_sStkNPS_setValue = CLIB.btDbvt_sStkNPS_setValue,
	MultiBodySolverConstraint_setMultiBodyA = CLIB.btMultiBodySolverConstraint_setMultiBodyA,
	MultiBodySolverConstraint_setLinkB = CLIB.btMultiBodySolverConstraint_setLinkB,
	WheelInfo_getMaxSuspensionForce = CLIB.btWheelInfo_getMaxSuspensionForce,
	SliderConstraint_setRestitutionLimAng = CLIB.btSliderConstraint_setRestitutionLimAng,
	CollisionWorld_serialize = CLIB.btCollisionWorld_serialize,
	BvhTriangleMeshShape_usesQuantizedAabbCompression = CLIB.btBvhTriangleMeshShape_usesQuantizedAabbCompression,
	MultiBodySolverConstraint_setJacBindex = CLIB.btMultiBodySolverConstraint_setJacBindex,
	GImpactBvh_boxQueryTrans = CLIB.btGImpactBvh_boxQueryTrans,
	PolyhedralConvexShape_getVertex = CLIB.btPolyhedralConvexShape_getVertex,
	SoftBody_sRayCast_new = CLIB.btSoftBody_sRayCast_new,
	AlignedObjectArray_btSoftBody_Anchor_size = CLIB.btAlignedObjectArray_btSoftBody_Anchor_size,
	CylinderShape_getHalfExtentsWithoutMargin = CLIB.btCylinderShape_getHalfExtentsWithoutMargin,
	SoftBody_SContact_new = CLIB.btSoftBody_SContact_new,
	MultiBodySolverConstraint_setDeltaVelBindex = CLIB.btMultiBodySolverConstraint_setDeltaVelBindex,
	GImpactMeshShapePart_getTrimeshPrimitiveManager = CLIB.btGImpactMeshShapePart_getTrimeshPrimitiveManager,
	CollisionWorld_ContactResultCallback_addSingleResult = CLIB.btCollisionWorld_ContactResultCallback_addSingleResult,
	MultiBodySolverConstraint_setDeltaVelAindex = CLIB.btMultiBodySolverConstraint_setDeltaVelAindex,
	MultiBodySolverConstraint_setContactNormal2 = CLIB.btMultiBodySolverConstraint_setContactNormal2,
	MultiBodySolverConstraint_setContactNormal1 = CLIB.btMultiBodySolverConstraint_setContactNormal1,
	QuantizedBvhNode_getEscapeIndexOrTriangleIndex = CLIB.btQuantizedBvhNode_getEscapeIndexOrTriangleIndex,
	MultiBodySolverConstraint_setAppliedPushImpulse = CLIB.btMultiBodySolverConstraint_setAppliedPushImpulse,
	MultiBodySolverConstraint_setAppliedImpulse = CLIB.btMultiBodySolverConstraint_setAppliedImpulse,
	MultiBodySolverConstraint_setAngularComponentA = CLIB.btMultiBodySolverConstraint_setAngularComponentA,
	ConvexCast_CastResult_setHitTransformA = CLIB.btConvexCast_CastResult_setHitTransformA,
	HingeConstraint_getUseFrameOffset = CLIB.btHingeConstraint_getUseFrameOffset,
	MultiBodySolverConstraint_getUpperLimit = CLIB.btMultiBodySolverConstraint_getUpperLimit,
	MultiBodySolverConstraint_getUnusedPadding4 = CLIB.btMultiBodySolverConstraint_getUnusedPadding4,
	RotationalLimitMotor2_setMotorCFM = CLIB.btRotationalLimitMotor2_setMotorCFM,
	ContactSolverInfoData_setMaxErrorReduction = CLIB.btContactSolverInfoData_setMaxErrorReduction,
	CompoundShape_getUpdateRevision = CLIB.btCompoundShape_getUpdateRevision,
	MultiBodySolverConstraint_getSolverBodyIdA = CLIB.btMultiBodySolverConstraint_getSolverBodyIdA,
	MultiBodySolverConstraint_getRhsPenetration = CLIB.btMultiBodySolverConstraint_getRhsPenetration,
	QuantizedBvh_new = CLIB.btQuantizedBvh_new,
	BoxBoxDetector_setBox2 = CLIB.btBoxBoxDetector_setBox2,
	MultiBodySolverConstraint_getRelpos2CrossNormal = CLIB.btMultiBodySolverConstraint_getRelpos2CrossNormal,
	Generic6DofConstraint_getAxis = CLIB.btGeneric6DofConstraint_getAxis,
	BroadphaseInterface_rayTest3 = CLIB.btBroadphaseInterface_rayTest3,
	Dbvt_ICollide_delete = CLIB.btDbvt_ICollide_delete,
	ContactSolverInfoData_setGlobalCfm = CLIB.btContactSolverInfoData_setGlobalCfm,
	AxisSweep3_testAabbOverlap = CLIB.btAxisSweep3_testAabbOverlap,
	MultiBodySolverConstraint_getMultiBodyA = CLIB.btMultiBodySolverConstraint_getMultiBodyA,
	SoftBodyHelpers_CreatePatchUV2 = CLIB.btSoftBodyHelpers_CreatePatchUV2,
	MultiBodySolverConstraint_getLinkB = CLIB.btMultiBodySolverConstraint_getLinkB,
	MultiBodySolverConstraint_getLinkA = CLIB.btMultiBodySolverConstraint_getLinkA,
	MultiBodySolverConstraint_getJacBindex = CLIB.btMultiBodySolverConstraint_getJacBindex,
	DbvtNode_setDataAsInt = CLIB.btDbvtNode_setDataAsInt,
	ConvexPlaneCollisionAlgorithm_CreateFunc_new = CLIB.btConvexPlaneCollisionAlgorithm_CreateFunc_new,
	SliderConstraint_getRestitutionLimLin = CLIB.btSliderConstraint_getRestitutionLimLin,
	DbvtBroadphase_optimize = CLIB.btDbvtBroadphase_optimize,
	MultiBodySolverConstraint_getContactNormal1 = CLIB.btMultiBodySolverConstraint_getContactNormal1,
	ManifoldResult_getPersistentManifold = CLIB.btManifoldResult_getPersistentManifold,
	MultiBodySolverConstraint_getAppliedPushImpulse = CLIB.btMultiBodySolverConstraint_getAppliedPushImpulse,
	MultiBody_getAngularDamping = CLIB.btMultiBody_getAngularDamping,
	MultiBodySolverConstraint_getAppliedImpulse = CLIB.btMultiBodySolverConstraint_getAppliedImpulse,
	MultiBodySolverConstraint_getAngularComponentB = CLIB.btMultiBodySolverConstraint_getAngularComponentB,
	Dbvt_sStkNN_getB = CLIB.btDbvt_sStkNN_getB,
	SoftBody_setBUpdateRtCst = CLIB.btSoftBody_setBUpdateRtCst,
	MultiBodyPoint2Point_setPivotInB = CLIB.btMultiBodyPoint2Point_setPivotInB,
	MultiBodyPoint2Point_getPivotInB = CLIB.btMultiBodyPoint2Point_getPivotInB,
	ConeTwistConstraint_getMotorTarget = CLIB.btConeTwistConstraint_getMotorTarget,
	ConeShape_getConeUpIndex = CLIB.btConeShape_getConeUpIndex,
	AlignedObjectArray_btSoftBody_Face_at = CLIB.btAlignedObjectArray_btSoftBody_Face_at,
	SoftBody_Impulse_getAsVelocity = CLIB.btSoftBody_Impulse_getAsVelocity,
	MultiBodyLinkCollider_setLink = CLIB.btMultiBodyLinkCollider_setLink,
	MultiBodyLinkCollider_getMultiBody = CLIB.btMultiBodyLinkCollider_getMultiBody,
	HingeConstraint_getEnableAngularMotor = CLIB.btHingeConstraint_getEnableAngularMotor,
	CollisionWorld_ContactResultCallback_needsCollision = CLIB.btCollisionWorld_ContactResultCallback_needsCollision,
	DbvtAabbMm_SignedExpand = CLIB.btDbvtAabbMm_SignedExpand,
	MultiBodyLinkCollider_new = CLIB.btMultiBodyLinkCollider_new,
	RaycastVehicle_addWheel = CLIB.btRaycastVehicle_addWheel,
	MultibodyLink_setZeroRotParentToThis = CLIB.btMultibodyLink_setZeroRotParentToThis,
	MultiBody_setWorldToBaseRot = CLIB.btMultiBody_setWorldToBaseRot,
	ConstraintSolver_delete = CLIB.btConstraintSolver_delete,
	Generic6DofConstraint_get_limit_motor_info22 = CLIB.btGeneric6DofConstraint_get_limit_motor_info22,
	MultibodyLink_setParent = CLIB.btMultibodyLink_setParent,
	SoftBody_updatePose = CLIB.btSoftBody_updatePose,
	GhostObject_addOverlappingObjectInternal2 = CLIB.btGhostObject_addOverlappingObjectInternal2,
	MultibodyLink_setLinkName = CLIB.btMultibodyLink_setLinkName,
	MultibodyLink_setJointType = CLIB.btMultibodyLink_setJointType,
	MultibodyLink_setJointName = CLIB.btMultibodyLink_setJointName,
	ConvexShape_localGetSupportingVertexWithoutMargin = CLIB.btConvexShape_localGetSupportingVertexWithoutMargin,
	SoftBody_Cluster_getCom = CLIB.btSoftBody_Cluster_getCom,
	Generic6DofSpring2Constraint_enableMotor = CLIB.btGeneric6DofSpring2Constraint_enableMotor,
	MultibodyLink_setJointFeedback = CLIB.btMultibodyLink_setJointFeedback,
	AlignedObjectArray_btSoftBody_Face_push_back = CLIB.btAlignedObjectArray_btSoftBody_Face_push_back,
	SoftBody_Config_getKDP = CLIB.btSoftBody_Config_getKDP,
	ConeTwistConstraint_isMotorEnabled = CLIB.btConeTwistConstraint_isMotorEnabled,
	CollisionObjectWrapper_setShape = CLIB.btCollisionObjectWrapper_setShape,
	DefaultMotionState_getUserPointer = CLIB.btDefaultMotionState_getUserPointer,
	ConeTwistConstraint_calcAngleInfo = CLIB.btConeTwistConstraint_calcAngleInfo,
	CollisionWorld_ContactResultCallback_getCollisionFilterMask = CLIB.btCollisionWorld_ContactResultCallback_getCollisionFilterMask,
	SoftBody_Config_getCiterations = CLIB.btSoftBody_Config_getCiterations,
	DefaultCollisionConstructionInfo_delete = CLIB.btDefaultCollisionConstructionInfo_delete,
	Dispatcher_needsResponse = CLIB.btDispatcher_needsResponse,
	DynamicsWorld_setInternalTickCallback2 = CLIB.btDynamicsWorld_setInternalTickCallback2,
	RotationalLimitMotor2_getEnableSpring = CLIB.btRotationalLimitMotor2_getEnableSpring,
	RigidBody_btRigidBodyConstructionInfo_getMotionState = CLIB.btRigidBody_btRigidBodyConstructionInfo_getMotionState,
	DefaultCollisionConstructionInfo_getPersistentManifoldPool = CLIB.btDefaultCollisionConstructionInfo_getPersistentManifoldPool,
	HingeAccumulatedAngleConstraint_new7 = CLIB.btHingeAccumulatedAngleConstraint_new7,
	ConvexPenetrationDepthSolver_calcPenDepth = CLIB.btConvexPenetrationDepthSolver_calcPenDepth,
	AlignedObjectArray_btSoftBody_ClusterPtr_resizeNoInitialize = CLIB.btAlignedObjectArray_btSoftBody_ClusterPtr_resizeNoInitialize,
	Box2dShape_getHalfExtentsWithMargin = CLIB.btBox2dShape_getHalfExtentsWithMargin,
	AngularLimit_set4 = CLIB.btAngularLimit_set4,
	AlignedObjectArray_btPersistentManifoldPtr_at = CLIB.btAlignedObjectArray_btPersistentManifoldPtr_at,
	DefaultCollisionConstructionInfo_getCollisionAlgorithmPool = CLIB.btDefaultCollisionConstructionInfo_getCollisionAlgorithmPool,
	Face_getIndices = CLIB.btFace_getIndices,
	MultiBody_setPosUpdated = CLIB.btMultiBody_setPosUpdated,
	BroadphaseProxy_getCollisionFilterGroup = CLIB.btBroadphaseProxy_getCollisionFilterGroup,
	Dbvt_remove = CLIB.btDbvt_remove,
	MultiBodySolverConstraint_getOverrideNumSolverIterations = CLIB.btMultiBodySolverConstraint_getOverrideNumSolverIterations,
	Hinge2Constraint_new = CLIB.btHinge2Constraint_new,
	RigidBody_getLocalInertia = CLIB.btRigidBody_getLocalInertia,
	GImpactQuantizedBvh_setPrimitiveManager = CLIB.btGImpactQuantizedBvh_setPrimitiveManager,
	DbvtBroadphase_getDupdates = CLIB.btDbvtBroadphase_getDupdates,
	MaterialProperties_getMaterialBase = CLIB.btMaterialProperties_getMaterialBase,
	UniversalConstraint_getAxis1 = CLIB.btUniversalConstraint_getAxis1,
	DbvtBroadphase_getNewpairs = CLIB.btDbvtBroadphase_getNewpairs,
	DefaultCollisionConfiguration_setConvexConvexMultipointIterations2 = CLIB.btDefaultCollisionConfiguration_setConvexConvexMultipointIterations2,
	Dbvt_sStkNP_setMask = CLIB.btDbvt_sStkNP_setMask,
	CompoundShapeChild_getNode = CLIB.btCompoundShapeChild_getNode,
	ConvexPlaneCollisionAlgorithm_CreateFunc_getNumPerturbationIterations = CLIB.btConvexPlaneCollisionAlgorithm_CreateFunc_getNumPerturbationIterations,
	DbvtBroadphase_getNeedcleanup = CLIB.btDbvtBroadphase_getNeedcleanup,
	CompoundShapeChild_setChildShape = CLIB.btCompoundShapeChild_setChildShape,
	SoftBody_setWorldInfo = CLIB.btSoftBody_setWorldInfo,
	CollisionShape_getContactBreakingThreshold = CLIB.btCollisionShape_getContactBreakingThreshold,
	AlignedObjectArray_btSoftBody_Node_at = CLIB.btAlignedObjectArray_btSoftBody_Node_at,
	Dbvt_benchmark = CLIB.btDbvt_benchmark,
	MultiBody_getNumPosVars = CLIB.btMultiBody_getNumPosVars,
	MultiBody_getJointPos = CLIB.btMultiBody_getJointPos,
	MultiBody_getBaseTorque = CLIB.btMultiBody_getBaseTorque,
	Dbvt_sStkNPS_getValue = CLIB.btDbvt_sStkNPS_getValue,
	DynamicsWorld_addRigidBody2 = CLIB.btDynamicsWorld_addRigidBody2,
	Dbvt_sStkNP_setNode = CLIB.btDbvt_sStkNP_setNode,
	DynamicsWorld_setInternalTickCallback = CLIB.btDynamicsWorld_setInternalTickCallback,
	MultiBody_fillContactJacobianMultiDof = CLIB.btMultiBody_fillContactJacobianMultiDof,
	DefaultCollisionConstructionInfo_getCustomCollisionAlgorithmMaxElementSize = CLIB.btDefaultCollisionConstructionInfo_getCustomCollisionAlgorithmMaxElementSize,
	TypedConstraint_btConstraintInfo2_getCfm = CLIB.btTypedConstraint_btConstraintInfo2_getCfm,
	MultiBodyDynamicsWorld_getMultiBodyConstraint = CLIB.btMultiBodyDynamicsWorld_getMultiBodyConstraint,
	ContactConstraint_setContactManifold = CLIB.btContactConstraint_setContactManifold,
	SliderConstraint_getSoftnessLimAng = CLIB.btSliderConstraint_getSoftnessLimAng,
	Dbvt_sStkNN_new = CLIB.btDbvt_sStkNN_new,
	CollisionShape_isPolyhedral = CLIB.btCollisionShape_isPolyhedral,
	Dbvt_sStkCLN_setNode = CLIB.btDbvt_sStkCLN_setNode,
	CompoundCompoundCollisionAlgorithm_new = CLIB.btCompoundCompoundCollisionAlgorithm_new,
	StridingMeshInterface_getLockedReadOnlyVertexIndexBase2 = CLIB.btStridingMeshInterface_getLockedReadOnlyVertexIndexBase2,
	GImpactCollisionAlgorithm_gimpact_vs_compoundshape = CLIB.btGImpactCollisionAlgorithm_gimpact_vs_compoundshape,
	DbvtNode_getChilds = CLIB.btDbvtNode_getChilds,
	DbvtBroadphase_getFupdates = CLIB.btDbvtBroadphase_getFupdates,
	RotationalLimitMotor_getBounce = CLIB.btRotationalLimitMotor_getBounce,
	DefaultSoftBodySolver_new = CLIB.btDefaultSoftBodySolver_new,
	CompoundCollisionAlgorithm_getChildAlgorithm = CLIB.btCompoundCollisionAlgorithm_getChildAlgorithm,
	MinkowskiSumShape_getTransformA = CLIB.btMinkowskiSumShape_getTransformA,
	ContactSolverInfoData_getTimeStep = CLIB.btContactSolverInfoData_getTimeStep,
	ConvexCast_CastResult_setDebugDrawer = CLIB.btConvexCast_CastResult_setDebugDrawer,
	Generic6DofSpring2Constraint_setEquilibriumPoint2 = CLIB.btGeneric6DofSpring2Constraint_setEquilibriumPoint2,
	WheelInfo_getClippedInvContactDotSuspension = CLIB.btWheelInfo_getClippedInvContactDotSuspension,
	CollisionWorld_RayResultCallback_setCollisionFilterGroup = CLIB.btCollisionWorld_RayResultCallback_setCollisionFilterGroup,
	BroadphasePair_getPProxy0 = CLIB.btBroadphasePair_getPProxy0,
	ContactSolverInfoData_setFriction = CLIB.btContactSolverInfoData_setFriction,
	BvhTriangleMeshShape_setOptimizedBvh2 = CLIB.btBvhTriangleMeshShape_setOptimizedBvh2,
	CollisionAlgorithmConstructionInfo_getDispatcher1 = CLIB.btCollisionAlgorithmConstructionInfo_getDispatcher1,
	RotationalLimitMotor2_setBounce = CLIB.btRotationalLimitMotor2_setBounce,
	Dbvt_setLeaves = CLIB.btDbvt_setLeaves,
	BroadphaseProxy_isSoftBody = CLIB.btBroadphaseProxy_isSoftBody,
	DynamicsWorld_getGravity = CLIB.btDynamicsWorld_getGravity,
	ManifoldPoint_getLocalPointB = CLIB.btManifoldPoint_getLocalPointB,
	AngularLimit_getHalfRange = CLIB.btAngularLimit_getHalfRange,
	ManifoldPoint_getLifeTime = CLIB.btManifoldPoint_getLifeTime,
	BvhTriangleMeshShape_setTriangleInfoMap = CLIB.btBvhTriangleMeshShape_setTriangleInfoMap,
	DbvtBroadphase_getPrediction = CLIB.btDbvtBroadphase_getPrediction,
	CollisionObject_getFriction = CLIB.btCollisionObject_getFriction,
	SliderConstraint_setFrames = CLIB.btSliderConstraint_setFrames,
	ConvexPolyhedron_setLocalCenter = CLIB.btConvexPolyhedron_setLocalCenter,
	RigidBody_proceedToTransform = CLIB.btRigidBody_proceedToTransform,
	SoftBody_initializeFaceTree = CLIB.btSoftBody_initializeFaceTree,
	MultiBody_isUsingGlobalVelocities = CLIB.btMultiBody_isUsingGlobalVelocities,
	ConvexHullShape_getNumPoints = CLIB.btConvexHullShape_getNumPoints,
	ManifoldPoint_getAppliedImpulseLateral1 = CLIB.btManifoldPoint_getAppliedImpulseLateral1,
	ContactSolverInfoData_getSplitImpulsePenetrationThreshold = CLIB.btContactSolverInfoData_getSplitImpulsePenetrationThreshold,
	ConvexInternalShape_setSafeMargin3 = CLIB.btConvexInternalShape_setSafeMargin3,
	ConvexPolyhedron_getMC = CLIB.btConvexPolyhedron_getMC,
	ConeTwistConstraint_setMaxMotorImpulseNormalized = CLIB.btConeTwistConstraint_setMaxMotorImpulseNormalized,
	MultibodyLink_getLinkName = CLIB.btMultibodyLink_getLinkName,
	ConvexPolyhedron_getLocalCenter = CLIB.btConvexPolyhedron_getLocalCenter,
	CollisionWorld_contactPairTest = CLIB.btCollisionWorld_contactPairTest,
	CollisionObject_hasAnisotropicFriction2 = CLIB.btCollisionObject_hasAnisotropicFriction2,
	ConvexPointCloudShape_setPoints2 = CLIB.btConvexPointCloudShape_setPoints2,
	AABB_appy_transform_trans_cache = CLIB.btAABB_appy_transform_trans_cache,
	CollisionObject_setIslandTag = CLIB.btCollisionObject_setIslandTag,
	CompoundShapeChild_setTransform = CLIB.btCompoundShapeChild_setTransform,
	CollisionWorld_RayResultCallback_setCollisionObject = CLIB.btCollisionWorld_RayResultCallback_setCollisionObject,
	Dbvt_update2 = CLIB.btDbvt_update2,
	SortedOverlappingPairCache_getOverlapFilterCallback = CLIB.btSortedOverlappingPairCache_getOverlapFilterCallback,
	AlignedObjectArray_btSoftBody_Anchor_push_back = CLIB.btAlignedObjectArray_btSoftBody_Anchor_push_back,
	WheelInfo_getBrake = CLIB.btWheelInfo_getBrake,
	BvhTriangleMeshShape_new = CLIB.btBvhTriangleMeshShape_new,
	AlignedObjectArray_btBroadphasePair_at = CLIB.btAlignedObjectArray_btBroadphasePair_at,
	RotationalLimitMotor_getCurrentLimitError = CLIB.btRotationalLimitMotor_getCurrentLimitError,
	RigidBody_getLinearVelocity = CLIB.btRigidBody_getLinearVelocity,
	GImpactQuantizedBvh_new = CLIB.btGImpactQuantizedBvh_new,
	TypedConstraint_btConstraintInfo2_getJ1linearAxis = CLIB.btTypedConstraint_btConstraintInfo2_getJ1linearAxis,
	DefaultMotionState_setUserPointer = CLIB.btDefaultMotionState_setUserPointer,
	ConvexConvexAlgorithm_CreateFunc_getMinimumPointsPerturbationThreshold = CLIB.btConvexConvexAlgorithm_CreateFunc_getMinimumPointsPerturbationThreshold,
	TranslationalLimitMotor2_getCurrentLimit = CLIB.btTranslationalLimitMotor2_getCurrentLimit,
	ConvexConvexAlgorithm_new = CLIB.btConvexConvexAlgorithm_new,
	ConvexConvexAlgorithm_CreateFunc_setPdSolver = CLIB.btConvexConvexAlgorithm_CreateFunc_setPdSolver,
	ConvexPolyhedron_getRadius = CLIB.btConvexPolyhedron_getRadius,
	IndexedMesh_setIndexType = CLIB.btIndexedMesh_setIndexType,
	DbvtBroadphase_getUpdates_call = CLIB.btDbvtBroadphase_getUpdates_call,
	Dispatcher_allocateCollisionAlgorithm = CLIB.btDispatcher_allocateCollisionAlgorithm,
	SoftBody_Cluster_getLeaf = CLIB.btSoftBody_Cluster_getLeaf,
	HingeConstraint_getInfo2Internal = CLIB.btHingeConstraint_getInfo2Internal,
	CylinderShapeX_new2 = CLIB.btCylinderShapeX_new2,
	MultiBodyDynamicsWorld_integrateTransforms = CLIB.btMultiBodyDynamicsWorld_integrateTransforms,
	MultibodyLink_getCachedRVector = CLIB.btMultibodyLink_getCachedRVector,
	ConvexCast_CastResult_setNormal = CLIB.btConvexCast_CastResult_setNormal,
	HingeConstraint_enableAngularMotor = CLIB.btHingeConstraint_enableAngularMotor,
	GImpactShapeInterface_getGImpactShapeType = CLIB.btGImpactShapeInterface_getGImpactShapeType,
	ManifoldResult_getBody0Internal = CLIB.btManifoldResult_getBody0Internal,
	BroadphaseInterface_delete = CLIB.btBroadphaseInterface_delete,
	Dbvt_sStkNN_new2 = CLIB.btDbvt_sStkNN_new2,
	MultiBody_calcAccelerationDeltasMultiDof = CLIB.btMultiBody_calcAccelerationDeltasMultiDof,
	CompoundShape_getChildShape = CLIB.btCompoundShape_getChildShape,
	CollisionShape_setMargin = CLIB.btCollisionShape_setMargin,
	ConvexCast_CastResult_getHitPoint = CLIB.btConvexCast_CastResult_getHitPoint,
	CollisionObject_getRestitution = CLIB.btCollisionObject_getRestitution,
	SoftBody_getFaces = CLIB.btSoftBody_getFaces,
	VoronoiSimplexSolver_getCachedP1 = CLIB.btVoronoiSimplexSolver_getCachedP1,
	RigidBody_getLinearSleepingThreshold = CLIB.btRigidBody_getLinearSleepingThreshold,
	Convex2dConvex2dAlgorithm_CreateFunc_setPdSolver = CLIB.btConvex2dConvex2dAlgorithm_CreateFunc_setPdSolver,
	GjkPairDetector_setFixContactNormalDirection = CLIB.btGjkPairDetector_setFixContactNormalDirection,
	GImpactQuantizedBvh_getNodeCount = CLIB.btGImpactQuantizedBvh_getNodeCount,
	GhostObject_convexSweepTest2 = CLIB.btGhostObject_convexSweepTest2,
	ConvexInternalShape_getMarginNV = CLIB.btConvexInternalShape_getMarginNV,
	ManifoldPoint_setPartId0 = CLIB.btManifoldPoint_setPartId0,
	GImpactMeshShapePart_TrimeshPrimitiveManager_lock = CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_lock,
	GImpactMeshShapePart_TrimeshPrimitiveManager_getScale = CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_getScale,
	MultiBodySolverConstraint_new = CLIB.btMultiBodySolverConstraint_new,
	GjkPairDetector_getDegenerateSimplex = CLIB.btGjkPairDetector_getDegenerateSimplex,
	ContactSolverInfoData_setSplitImpulse = CLIB.btContactSolverInfoData_setSplitImpulse,
	ContactSolverInfoData_setSor = CLIB.btContactSolverInfoData_setSor,
	ConeTwistConstraint_getLimit = CLIB.btConeTwistConstraint_getLimit,
	RotationalLimitMotor2_getCurrentPosition = CLIB.btRotationalLimitMotor2_getCurrentPosition,
	ConvexPolyhedron_setME = CLIB.btConvexPolyhedron_setME,
	ConvexTriangleCallback_setTriangleCount = CLIB.btConvexTriangleCallback_setTriangleCount,
	ContactSolverInfoData_setSingleAxisRollingFrictionThreshold = CLIB.btContactSolverInfoData_setSingleAxisRollingFrictionThreshold,
	DbvtAabbMm_FromCR = CLIB.btDbvtAabbMm_FromCR,
	GhostObject_upcast = CLIB.btGhostObject_upcast,
	GImpactMeshShapePart_TrimeshPrimitiveManager_setNumfaces = CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_setNumfaces,
	Dbvt_optimizeTopDown = CLIB.btDbvt_optimizeTopDown,
	FixedConstraint_new = CLIB.btFixedConstraint_new,
	TriangleMeshShape_getLocalAabbMin = CLIB.btTriangleMeshShape_getLocalAabbMin,
	Convex2dConvex2dAlgorithm_CreateFunc_getSimplexSolver = CLIB.btConvex2dConvex2dAlgorithm_CreateFunc_getSimplexSolver,
	MultiBodySolverConstraint_getOrgConstraint = CLIB.btMultiBodySolverConstraint_getOrgConstraint,
	BroadphaseProxy_setCollisionFilterGroup = CLIB.btBroadphaseProxy_setCollisionFilterGroup,
	GImpactMeshShapePart_TrimeshPrimitiveManager_setIndexbase = CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_setIndexbase,
	ContactSolverInfoData_setErp2 = CLIB.btContactSolverInfoData_setErp2,
	GeometryUtil_getVerticesFromPlaneEquations = CLIB.btGeometryUtil_getVerticesFromPlaneEquations,
	GhostObject_getOverlappingPairs = CLIB.btGhostObject_getOverlappingPairs,
	ContactSolverInfoData_getWarmstartingFactor = CLIB.btContactSolverInfoData_getWarmstartingFactor,
	DefaultCollisionConstructionInfo_setDefaultMaxPersistentManifoldPoolSize = CLIB.btDefaultCollisionConstructionInfo_setDefaultMaxPersistentManifoldPoolSize,
	ContactSolverInfoData_getMaxGyroscopicForce = CLIB.btContactSolverInfoData_getMaxGyroscopicForce,
	ContactSolverInfoData_getMaxErrorReduction = CLIB.btContactSolverInfoData_getMaxErrorReduction,
	DefaultMotionState_getGraphicsWorldTrans = CLIB.btDefaultMotionState_getGraphicsWorldTrans,
	DefaultCollisionConstructionInfo_getDefaultMaxCollisionAlgorithmPoolSize = CLIB.btDefaultCollisionConstructionInfo_getDefaultMaxCollisionAlgorithmPoolSize,
	ContactSolverInfoData_getDamping = CLIB.btContactSolverInfoData_getDamping,
	Convex2dShape_getChildShape = CLIB.btConvex2dShape_getChildShape,
	Dbvt_extractLeaves = CLIB.btDbvt_extractLeaves,
	GImpactShapeInterface_setChildTransform = CLIB.btGImpactShapeInterface_setChildTransform,
	BroadphaseRayCallback_getRayDirectionInverse = CLIB.btBroadphaseRayCallback_getRayDirectionInverse,
	CollisionObjectWrapper_setCollisionObject = CLIB.btCollisionObjectWrapper_setCollisionObject,
	ConvexInternalShape_setSafeMargin = CLIB.btConvexInternalShape_setSafeMargin,
	CylinderShapeZ_new = CLIB.btCylinderShapeZ_new,
	DynamicsWorld_addRigidBody = CLIB.btDynamicsWorld_addRigidBody,
	ConeTwistConstraint_setLimit4 = CLIB.btConeTwistConstraint_setLimit4,
	KinematicCharacterController_setGravity = CLIB.btKinematicCharacterController_setGravity,
	CharacterControllerInterface_onGround = CLIB.btCharacterControllerInterface_onGround,
	SoftBodyHelpers_DrawFaceTree3 = CLIB.btSoftBodyHelpers_DrawFaceTree3,
	CylinderShape_getHalfExtentsWithMargin = CLIB.btCylinderShape_getHalfExtentsWithMargin,
	DefaultCollisionConfiguration_getSimplexSolver = CLIB.btDefaultCollisionConfiguration_getSimplexSolver,
	GImpactShapeInterface_getChildTransform = CLIB.btGImpactShapeInterface_getChildTransform,
	TranslationalLimitMotor_setRestitution = CLIB.btTranslationalLimitMotor_setRestitution,
	DiscreteCollisionDetectorInterface_delete = CLIB.btDiscreteCollisionDetectorInterface_delete,
	DispatcherInfo_getUseContinuous = CLIB.btDispatcherInfo_getUseContinuous,
	MultiBody_internalNeedsJointFeedback = CLIB.btMultiBody_internalNeedsJointFeedback,
	WorldImporter_createConeShapeZ = CLIB.btWorldImporter_createConeShapeZ,
	Chunk_getLength = CLIB.btChunk_getLength,
	TranslationalLimitMotor2_getMaxMotorForce = CLIB.btTranslationalLimitMotor2_getMaxMotorForce,
	CollisionObject_setBroadphaseHandle = CLIB.btCollisionObject_setBroadphaseHandle,
	GearConstraint_new2 = CLIB.btGearConstraint_new2,
	MultibodyLink_getJointFriction = CLIB.btMultibodyLink_getJointFriction,
	DbvtBroadphase_setFupdates = CLIB.btDbvtBroadphase_setFupdates,
	CollisionDispatcher_getNearCallback = CLIB.btCollisionDispatcher_getNearCallback,
	Dispatcher_delete = CLIB.btDispatcher_delete,
	CollisionObject_delete = CLIB.btCollisionObject_delete,
	Dbvt_ICollide_new = CLIB.btDbvt_ICollide_new,
	CompoundShape_removeChildShape = CLIB.btCompoundShape_removeChildShape,
	CollisionObject_new = CLIB.btCollisionObject_new,
	CollisionWorld_ConvexResultCallbackWrapper_new = CLIB.btCollisionWorld_ConvexResultCallbackWrapper_new,
	KinematicCharacterController_new2 = CLIB.btKinematicCharacterController_new2,
	ConeTwistConstraint_getFixThresh = CLIB.btConeTwistConstraint_getFixThresh,
	Dispatcher_getInternalManifoldPool = CLIB.btDispatcher_getInternalManifoldPool,
	Point2PointConstraint_new2 = CLIB.btPoint2PointConstraint_new2,
	ConeTwistConstraint_new = CLIB.btConeTwistConstraint_new,
	SoftBody_Link_getBbending = CLIB.btSoftBody_Link_getBbending,
	ConeShape_setConeUpIndex = CLIB.btConeShape_setConeUpIndex,
	QuantizedBvh_quantizeWithClamp = CLIB.btQuantizedBvh_quantizeWithClamp,
	RigidBody_btRigidBodyConstructionInfo_getAngularSleepingThreshold = CLIB.btRigidBody_btRigidBodyConstructionInfo_getAngularSleepingThreshold,
	GImpactMeshShapePart_TrimeshPrimitiveManager_setNumverts = CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_setNumverts,
	WheelInfoConstructionInfo_delete = CLIB.btWheelInfoConstructionInfo_delete,
	CompoundShape_recalculateLocalAabb = CLIB.btCompoundShape_recalculateLocalAabb,
	SimulationIslandManager_initUnionFind = CLIB.btSimulationIslandManager_initUnionFind,
	Generic6DofSpringConstraint_setEquilibriumPoint = CLIB.btGeneric6DofSpringConstraint_setEquilibriumPoint,
	CollisionWorld_LocalConvexResult_getHitFraction = CLIB.btCollisionWorld_LocalConvexResult_getHitFraction,
	AABB_new4 = CLIB.btAABB_new4,
	DiscreteDynamicsWorld_getSimulationIslandManager = CLIB.btDiscreteDynamicsWorld_getSimulationIslandManager,
	ManifoldPoint_setContactCFM = CLIB.btManifoldPoint_setContactCFM,
	CollisionObject_getCcdSquareMotionThreshold = CLIB.btCollisionObject_getCcdSquareMotionThreshold,
	CompoundShapeChild_setNode = CLIB.btCompoundShapeChild_setNode,
	ConvexShape_getMarginNonVirtual = CLIB.btConvexShape_getMarginNonVirtual,
	GImpactCollisionAlgorithm_setFace0 = CLIB.btGImpactCollisionAlgorithm_setFace0,
	BroadphaseAabbCallbackWrapper_new = CLIB.btBroadphaseAabbCallbackWrapper_new,
	CapsuleShape_getRadius = CLIB.btCapsuleShape_getRadius,
	CollisionObject_setRollingFriction = CLIB.btCollisionObject_setRollingFriction,
	GImpactQuantizedBvh_setNodeBound = CLIB.btGImpactQuantizedBvh_setNodeBound,
	CompoundShapeChild_getTransform = CLIB.btCompoundShapeChild_getTransform,
	BroadphaseInterface_resetPool = CLIB.btBroadphaseInterface_resetPool,
	SoftBody_getClusterConnectivity = CLIB.btSoftBody_getClusterConnectivity,
	MultibodyLink_getZeroRotParentToThis = CLIB.btMultibodyLink_getZeroRotParentToThis,
	TriangleMeshShape_recalcLocalAabb = CLIB.btTriangleMeshShape_recalcLocalAabb,
	Generic6DofSpring2Constraint_setStiffness = CLIB.btGeneric6DofSpring2Constraint_setStiffness,
	CompoundCompoundCollisionAlgorithm_CreateFunc_new = CLIB.btCompoundCompoundCollisionAlgorithm_CreateFunc_new,
	TypedConstraint_btConstraintInfo2_getDamping = CLIB.btTypedConstraint_btConstraintInfo2_getDamping,
	CollisionObject_setCcdMotionThreshold = CLIB.btCollisionObject_setCcdMotionThreshold,
	TriangleIndexVertexArray_new2 = CLIB.btTriangleIndexVertexArray_new2,
	TypedConstraint_setUserConstraintId = CLIB.btTypedConstraint_setUserConstraintId,
	DiscreteCollisionDetectorInterface_ClosestPointInput_getMaximumDistanceSquared = CLIB.btDiscreteCollisionDetectorInterface_ClosestPointInput_getMaximumDistanceSquared,
	Dbvt_ICollide_Process2 = CLIB.btDbvt_ICollide_Process2,
	GImpactBvh_new2 = CLIB.btGImpactBvh_new2,
	QuantizedBvhTree_getLeftNode = CLIB.btQuantizedBvhTree_getLeftNode,
	SoftBody_Pose_getVolume = CLIB.btSoftBody_Pose_getVolume,
	ConstraintSetting_getTau = CLIB.btConstraintSetting_getTau,
	GImpactMeshShapePart_TrimeshPrimitiveManager_setVertexbase = CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_setVertexbase,
	SoftBodyHelpers_DrawClusterTree3 = CLIB.btSoftBodyHelpers_DrawClusterTree3,
	SoftBody_SContact_getWeights = CLIB.btSoftBody_SContact_getWeights,
	SoftBodyHelpers_CreateFromConvexHull = CLIB.btSoftBodyHelpers_CreateFromConvexHull,
	Dbvt_IWriter_Prepare = CLIB.btDbvt_IWriter_Prepare,
	AlignedObjectArray_btSoftBody_Link_resizeNoInitialize = CLIB.btAlignedObjectArray_btSoftBody_Link_resizeNoInitialize,
	CollisionShape_setLocalScaling = CLIB.btCollisionShape_setLocalScaling,
	SoftBody_getLinkVertexNormalData = CLIB.btSoftBody_getLinkVertexNormalData,
	SoftBody_addAeroForceToNode = CLIB.btSoftBody_addAeroForceToNode,
	Generic6DofConstraint_get_limit_motor_info2 = CLIB.btGeneric6DofConstraint_get_limit_motor_info2,
	CollisionWorld_LocalRayResult_new = CLIB.btCollisionWorld_LocalRayResult_new,
	GImpactShapeInterface_getPrimitiveTriangle = CLIB.btGImpactShapeInterface_getPrimitiveTriangle,
	MultiBodyPoint2Point_new2 = CLIB.btMultiBodyPoint2Point_new2,
	AABB_overlapping_trans_cache = CLIB.btAABB_overlapping_trans_cache,
	PolarDecomposition_new2 = CLIB.btPolarDecomposition_new2,
	CollisionWorld_LocalConvexResult_getHitNormalLocal = CLIB.btCollisionWorld_LocalConvexResult_getHitNormalLocal,
	Generic6DofSpringConstraint_getStiffness = CLIB.btGeneric6DofSpringConstraint_getStiffness,
	CollisionWorld_ConvexResultCallback_getCollisionFilterGroup = CLIB.btCollisionWorld_ConvexResultCallback_getCollisionFilterGroup,
	ManifoldResult_refreshContactPoints = CLIB.btManifoldResult_refreshContactPoints,
	CollisionWorld_addCollisionObject2 = CLIB.btCollisionWorld_addCollisionObject2,
	CollisionWorld_ClosestConvexResultCallback_setConvexFromWorld = CLIB.btCollisionWorld_ClosestConvexResultCallback_setConvexFromWorld,
	ConvexCast_delete = CLIB.btConvexCast_delete,
	SoftBodySolver_getTimeScale = CLIB.btSoftBodySolver_getTimeScale,
	Generic6DofSpring2Constraint_getAngularUpperLimitReversed = CLIB.btGeneric6DofSpring2Constraint_getAngularUpperLimitReversed,
	Dbvt_sStkNPS_new2 = CLIB.btDbvt_sStkNPS_new2,
	ConvexPointCloudShape_setPoints = CLIB.btConvexPointCloudShape_setPoints,
	CollisionWorld_contactTest = CLIB.btCollisionWorld_contactTest,
	DbvtAabbMm_Center = CLIB.btDbvtAabbMm_Center,
	MultiBody_getNumDofs = CLIB.btMultiBody_getNumDofs,
	Dbvt_allocate = CLIB.btDbvt_allocate,
	CollisionWorld_ClosestConvexResultCallback_setHitPointWorld = CLIB.btCollisionWorld_ClosestConvexResultCallback_setHitPointWorld,
	CollisionWorld_ClosestConvexResultCallback_setHitNormalWorld = CLIB.btCollisionWorld_ClosestConvexResultCallback_setHitNormalWorld,
	SoftBodySolver_solveConstraints = CLIB.btSoftBodySolver_solveConstraints,
	CollisionWorld_objectQuerySingle = CLIB.btCollisionWorld_objectQuerySingle,
	DbvtAabbMm_tMins = CLIB.btDbvtAabbMm_tMins,
	Generic6DofSpringConstraint_setDamping = CLIB.btGeneric6DofSpringConstraint_setDamping,
	CollisionWorld_ClosestConvexResultCallback_getHitPointWorld = CLIB.btCollisionWorld_ClosestConvexResultCallback_getHitPointWorld,
	CollisionConfiguration_getPersistentManifoldPool = CLIB.btCollisionConfiguration_getPersistentManifoldPool,
	OptimizedBvh_refitPartial = CLIB.btOptimizedBvh_refitPartial,
	SoftBody_addForce2 = CLIB.btSoftBody_addForce2,
	HingeAccumulatedAngleConstraint_new8 = CLIB.btHingeAccumulatedAngleConstraint_new8,
	WorldImporter_createTriangleInfoMap = CLIB.btWorldImporter_createTriangleInfoMap,
	AxisSweep3_setOverlappingPairUserCallback = CLIB.btAxisSweep3_setOverlappingPairUserCallback,
	ConeShape_getHeight = CLIB.btConeShape_getHeight,
	RaycastVehicle_btVehicleTuning_getSuspensionStiffness = CLIB.btRaycastVehicle_btVehicleTuning_getSuspensionStiffness,
	BroadphasePair_delete = CLIB.btBroadphasePair_delete,
	SoftBody_defaultCollisionHandler = CLIB.btSoftBody_defaultCollisionHandler,
	RotationalLimitMotor2_getMotorERP = CLIB.btRotationalLimitMotor2_getMotorERP,
	Generic6DofSpring2Constraint_getLinearUpperLimit = CLIB.btGeneric6DofSpring2Constraint_getLinearUpperLimit,
	ManifoldPoint_getDistance1 = CLIB.btManifoldPoint_getDistance1,
	Generic6DofSpring2Constraint_setEquilibriumPoint3 = CLIB.btGeneric6DofSpring2Constraint_setEquilibriumPoint3,
	CollisionObject_getDeactivationTime = CLIB.btCollisionObject_getDeactivationTime,
	CollisionObject_setRestitution = CLIB.btCollisionObject_setRestitution,
	RotationalLimitMotor2_setSpringDampingLimited = CLIB.btRotationalLimitMotor2_setSpringDampingLimited,
	BoxBoxDetector_setBox1 = CLIB.btBoxBoxDetector_setBox1,
	CollisionWorld_LocalRayResult_setHitFraction = CLIB.btCollisionWorld_LocalRayResult_setHitFraction,
	CollisionObject_internalGetExtensionPointer = CLIB.btCollisionObject_internalGetExtensionPointer,
	MultiBodyConstraint_getIslandIdA = CLIB.btMultiBodyConstraint_getIslandIdA,
	PointCollector_setPointInWorld = CLIB.btPointCollector_setPointInWorld,
	Generic6DofSpring2Constraint_getAngle = CLIB.btGeneric6DofSpring2Constraint_getAngle,
	QuantizedBvhTree_getNodeBound = CLIB.btQuantizedBvhTree_getNodeBound,
	DefaultCollisionConfiguration_setPlaneConvexMultipointIterations = CLIB.btDefaultCollisionConfiguration_setPlaneConvexMultipointIterations,
	CompoundShape_addChildShape = CLIB.btCompoundShape_addChildShape,
	SoftBody_evaluateCom = CLIB.btSoftBody_evaluateCom,
	ConeTwistConstraint_getInfo2NonVirtual = CLIB.btConeTwistConstraint_getInfo2NonVirtual,
	BoxShape_getPlaneEquation = CLIB.btBoxShape_getPlaneEquation,
	GImpactBvh_update = CLIB.btGImpactBvh_update,
	AlignedObjectArray_btSoftBody_Link_push_back = CLIB.btAlignedObjectArray_btSoftBody_Link_push_back,
	CollisionObject_setInterpolationWorldTransform = CLIB.btCollisionObject_setInterpolationWorldTransform,
	Convex2dConvex2dAlgorithm_CreateFunc_getNumPerturbationIterations = CLIB.btConvex2dConvex2dAlgorithm_CreateFunc_getNumPerturbationIterations,
	ConvexShape_getAabbSlow = CLIB.btConvexShape_getAabbSlow,
	MultibodyLink_getAbsFrameTotVelocity = CLIB.btMultibodyLink_getAbsFrameTotVelocity,
	CollisionWorld_LocalConvexResult_setLocalShapeInfo = CLIB.btCollisionWorld_LocalConvexResult_setLocalShapeInfo,
	MultiBody_setMaxAppliedImpulse = CLIB.btMultiBody_setMaxAppliedImpulse,
	TranslationalLimitMotor2_getBounce = CLIB.btTranslationalLimitMotor2_getBounce,
	BvhTriangleMeshShape_new3 = CLIB.btBvhTriangleMeshShape_new3,
	MultiBodyConstraint_delete = CLIB.btMultiBodyConstraint_delete,
	GImpactMeshShapePart_TrimeshPrimitiveManager_getLock_count = CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_getLock_count,
	SliderConstraint_setRestitutionLimLin = CLIB.btSliderConstraint_setRestitutionLimLin,
	CollisionWorld_ClosestRayResultCallback_getHitPointWorld = CLIB.btCollisionWorld_ClosestRayResultCallback_getHitPointWorld,
	KinematicCharacterController_setMaxSlope = CLIB.btKinematicCharacterController_setMaxSlope,
	PairSet_push_pair_inv = CLIB.btPairSet_push_pair_inv,
	CollisionObjectWrapper_getIndex = CLIB.btCollisionObjectWrapper_getIndex,
	ConeTwistConstraint_setMaxMotorImpulse = CLIB.btConeTwistConstraint_setMaxMotorImpulse,
	DbvtBroadphase_getCid = CLIB.btDbvtBroadphase_getCid,
	Hinge2Constraint_setLowerLimit = CLIB.btHinge2Constraint_setLowerLimit,
	BroadphaseInterface_printStats = CLIB.btBroadphaseInterface_printStats,
	BoxBoxCollisionAlgorithm_new = CLIB.btBoxBoxCollisionAlgorithm_new,
	CollisionWorld_addCollisionObject = CLIB.btCollisionWorld_addCollisionObject,
	AlignedObjectArray_btBroadphasePair_size = CLIB.btAlignedObjectArray_btBroadphasePair_size,
	RigidBody_applyGravity = CLIB.btRigidBody_applyGravity,
	ContactSolverInfoData_getSolverMode = CLIB.btContactSolverInfoData_getSolverMode,
	TriangleCallbackWrapper_new = CLIB.btTriangleCallbackWrapper_new,
	ConvexInternalShape_setSafeMargin2 = CLIB.btConvexInternalShape_setSafeMargin2,
	ConeTwistConstraint_getLimitSoftness = CLIB.btConeTwistConstraint_getLimitSoftness,
	BroadphaseProxy_getAabbMax = CLIB.btBroadphaseProxy_getAabbMax,
	Generic6DofSpring2Constraint_setLimit = CLIB.btGeneric6DofSpring2Constraint_setLimit,
	ManifoldPoint_getFrictionCFM = CLIB.btManifoldPoint_getFrictionCFM,
	MultibodyLink_setCollider = CLIB.btMultibodyLink_setCollider,
	AlignedObjectArray_btSoftBody_Face_size = CLIB.btAlignedObjectArray_btSoftBody_Face_size,
	ManifoldPoint_setDistance = CLIB.btManifoldPoint_setDistance,
	CollisionWorld_ConvexResultCallback_hasHit = CLIB.btCollisionWorld_ConvexResultCallback_hasHit,
	BroadphaseProxy_getUniqueId = CLIB.btBroadphaseProxy_getUniqueId,
	MultiBody_serialize = CLIB.btMultiBody_serialize,
	MultiBodyDynamicsWorld_new = CLIB.btMultiBodyDynamicsWorld_new,
	CollisionWorld_AllHitsRayResultCallback_setRayToWorld = CLIB.btCollisionWorld_AllHitsRayResultCallback_setRayToWorld,
	WheelInfo_getWheelsDampingRelaxation = CLIB.btWheelInfo_getWheelsDampingRelaxation,
	SparseSdf3_Initialize = CLIB.btSparseSdf3_Initialize,
	AlignedObjectArray_btSoftBody_MaterialPtr_at = CLIB.btAlignedObjectArray_btSoftBody_MaterialPtr_at,
	SoftBody_Tetra_getC2 = CLIB.btSoftBody_Tetra_getC2,
	DispatcherInfo_getDispatchFunc = CLIB.btDispatcherInfo_getDispatchFunc,
	AlignedObjectArray_btSoftBody_Face_resizeNoInitialize = CLIB.btAlignedObjectArray_btSoftBody_Face_resizeNoInitialize,
	HingeConstraint_new5 = CLIB.btHingeConstraint_new5,
	ConvexConvexAlgorithm_CreateFunc_getNumPerturbationIterations = CLIB.btConvexConvexAlgorithm_CreateFunc_getNumPerturbationIterations,
	DiscreteCollisionDetectorInterface_ClosestPointInput_delete = CLIB.btDiscreteCollisionDetectorInterface_ClosestPointInput_delete,
	RotationalLimitMotor2_getStopCFM = CLIB.btRotationalLimitMotor2_getStopCFM,
	CollisionWorld_AllHitsRayResultCallback_new = CLIB.btCollisionWorld_AllHitsRayResultCallback_new,
	DbvtBroadphase_performDeferredRemoval = CLIB.btDbvtBroadphase_performDeferredRemoval,
	CollisionObject_getInterpolationAngularVelocity = CLIB.btCollisionObject_getInterpolationAngularVelocity,
	Generic6DofSpringConstraint_setEquilibriumPoint2 = CLIB.btGeneric6DofSpringConstraint_setEquilibriumPoint2,
	ConvexPlaneCollisionAlgorithm_collideSingleContact = CLIB.btConvexPlaneCollisionAlgorithm_collideSingleContact,
	AABB_new3 = CLIB.btAABB_new3,
	TranslationalLimitMotor2_getCurrentLimitErrorHi = CLIB.btTranslationalLimitMotor2_getCurrentLimitErrorHi,
	CollisionObject_getHitFraction = CLIB.btCollisionObject_getHitFraction,
	GImpactShapeInterface_updateBound = CLIB.btGImpactShapeInterface_updateBound,
	CollisionWorld_AllHitsRayResultCallback_getHitPointWorld = CLIB.btCollisionWorld_AllHitsRayResultCallback_getHitPointWorld,
	CompoundCompoundCollisionAlgorithm_SwappedCreateFunc_new = CLIB.btCompoundCompoundCollisionAlgorithm_SwappedCreateFunc_new,
	QuantizedBvhNode_getQuantizedAabbMax = CLIB.btQuantizedBvhNode_getQuantizedAabbMax,
	CollisionShape_getUserPointer = CLIB.btCollisionShape_getUserPointer,
	GjkEpaPenetrationDepthSolver_new = CLIB.btGjkEpaPenetrationDepthSolver_new,
	CollisionObject_getActivationState = CLIB.btCollisionObject_getActivationState,
	SoftRigidCollisionAlgorithm_CreateFunc_new = CLIB.btSoftRigidCollisionAlgorithm_CreateFunc_new,
	GjkPairDetector_getCachedSeparatingDistance = CLIB.btGjkPairDetector_getCachedSeparatingDistance,
	CollisionAlgorithmCreateFunc_setSwapped = CLIB.btCollisionAlgorithmCreateFunc_setSwapped,
	CollisionObjectWrapper_setPartId = CLIB.btCollisionObjectWrapper_setPartId,
	TranslationalLimitMotor2_getStopERP = CLIB.btTranslationalLimitMotor2_getStopERP,
	CollisionObject_mergesSimulationIslands = CLIB.btCollisionObject_mergesSimulationIslands,
	PersistentManifold_removeContactPoint = CLIB.btPersistentManifold_removeContactPoint,
	ConvexTriangleCallback_getManifoldPtr = CLIB.btConvexTriangleCallback_getManifoldPtr,
	CollisionAlgorithmCreateFunc_getSwapped = CLIB.btCollisionAlgorithmCreateFunc_getSwapped,
	ConstraintSolver_getSolverType = CLIB.btConstraintSolver_getSolverType,
	Generic6DofConstraint_getInfo2NonVirtual = CLIB.btGeneric6DofConstraint_getInfo2NonVirtual,
	CollisionWorld_RayResultCallback_needsCollision = CLIB.btCollisionWorld_RayResultCallback_needsCollision,
	GImpactBvh_isTrimesh = CLIB.btGImpactBvh_isTrimesh,
	GImpactMeshShapePart_TrimeshPrimitiveManager_getPart = CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_getPart,
	HingeConstraint_setMotorTargetVelocity = CLIB.btHingeConstraint_setMotorTargetVelocity,
	CollisionWorld_ClosestRayResultCallback_setRayFromWorld = CLIB.btCollisionWorld_ClosestRayResultCallback_setRayFromWorld,
	Dbvt_ICollide_Process = CLIB.btDbvt_ICollide_Process,
	Generic6DofConstraint_getAngularUpperLimit = CLIB.btGeneric6DofConstraint_getAngularUpperLimit,
	ConvexPenetrationDepthSolver_delete = CLIB.btConvexPenetrationDepthSolver_delete,
	SoftBodyWorldInfo_getBroadphase = CLIB.btSoftBodyWorldInfo_getBroadphase,
	ConvexConvexAlgorithm_CreateFunc_setSimplexSolver = CLIB.btConvexConvexAlgorithm_CreateFunc_setSimplexSolver,
	TypedConstraint_solveConstraintObsolete = CLIB.btTypedConstraint_solveConstraintObsolete,
	TypedConstraint_getFixedBody = CLIB.btTypedConstraint_getFixedBody,
	BroadphaseProxy_getCollisionFilterMask = CLIB.btBroadphaseProxy_getCollisionFilterMask,
	CollisionWorld_ClosestRayResultCallback_setHitPointWorld = CLIB.btCollisionWorld_ClosestRayResultCallback_setHitPointWorld,
	KinematicCharacterController_getMaxSlope = CLIB.btKinematicCharacterController_getMaxSlope,
	SoftBodyRigidBodyCollisionConfiguration_new = CLIB.btSoftBodyRigidBodyCollisionConfiguration_new,
	RotationalLimitMotor_getMaxLimitForce = CLIB.btRotationalLimitMotor_getMaxLimitForce,
	GImpactMeshShapePart_TrimeshPrimitiveManager_setStride = CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_setStride,
	AABB_find_intersection = CLIB.btAABB_find_intersection,
	MultiBodyConstraint_allocateJacobiansMultiDof = CLIB.btMultiBodyConstraint_allocateJacobiansMultiDof,
	DiscreteDynamicsWorld_getApplySpeculativeContactRestitution = CLIB.btDiscreteDynamicsWorld_getApplySpeculativeContactRestitution,
	TranslationalLimitMotor_setLowerLimit = CLIB.btTranslationalLimitMotor_setLowerLimit,
	AABB_overlapping_trans_conservative = CLIB.btAABB_overlapping_trans_conservative,
	CollisionWorld_ClosestRayResultCallback_setHitNormalWorld = CLIB.btCollisionWorld_ClosestRayResultCallback_setHitNormalWorld,
	DbvtBroadphase_setDupdates = CLIB.btDbvtBroadphase_setDupdates,
	CompoundShape_new = CLIB.btCompoundShape_new,
	TranslationalLimitMotor_getRestitution = CLIB.btTranslationalLimitMotor_getRestitution,
	ConeTwistConstraint_getBFrame = CLIB.btConeTwistConstraint_getBFrame,
	SoftBody_RayFromToCaster_getRayNormalizedDirection = CLIB.btSoftBody_RayFromToCaster_getRayNormalizedDirection,
	ContactSolverInfoData_getSplitImpulse = CLIB.btContactSolverInfoData_getSplitImpulse,
	CollisionObjectWrapper_getCollisionObject = CLIB.btCollisionObjectWrapper_getCollisionObject,
	GearConstraint_getRatio = CLIB.btGearConstraint_getRatio,
	CollisionObject_getInterpolationLinearVelocity = CLIB.btCollisionObject_getInterpolationLinearVelocity,
	BroadphaseProxy_setCollisionFilterMask = CLIB.btBroadphaseProxy_setCollisionFilterMask,
	CollisionObject_getContactProcessingThreshold = CLIB.btCollisionObject_getContactProcessingThreshold,
	RotationalLimitMotor_setMaxMotorForce = CLIB.btRotationalLimitMotor_setMaxMotorForce,
	GearConstraint_getAxisB = CLIB.btGearConstraint_getAxisB,
	RotationalLimitMotor_setCurrentLimit = CLIB.btRotationalLimitMotor_setCurrentLimit,
	CollisionObject_getIslandTag = CLIB.btCollisionObject_getIslandTag,
	RotationalLimitMotor_setEnableMotor = CLIB.btRotationalLimitMotor_setEnableMotor,
	Generic6DofConstraint_getFlags = CLIB.btGeneric6DofConstraint_getFlags,
	GImpactCollisionAlgorithm_getPart0 = CLIB.btGImpactCollisionAlgorithm_getPart0,
	CollisionWorld_ConvexResultCallback_getCollisionFilterMask = CLIB.btCollisionWorld_ConvexResultCallback_getCollisionFilterMask,
	Generic6DofConstraint_getUseSolveConstraintObsolete = CLIB.btGeneric6DofConstraint_getUseSolveConstraintObsolete,
	BroadphaseInterface_calculateOverlappingPairs = CLIB.btBroadphaseInterface_calculateOverlappingPairs,
	VoronoiSimplexSolver_getCachedP2 = CLIB.btVoronoiSimplexSolver_getCachedP2,
	RigidBody_setNewBroadphaseProxy = CLIB.btRigidBody_setNewBroadphaseProxy,
	GearConstraint_new = CLIB.btGearConstraint_new,
	TypedConstraint_enableFeedback = CLIB.btTypedConstraint_enableFeedback,
	MultiSphereShape_new = CLIB.btMultiSphereShape_new,
	BroadphaseInterface_setAabb = CLIB.btBroadphaseInterface_setAabb,
	AABB_new5 = CLIB.btAABB_new5,
	RotationalLimitMotor_getLoLimit = CLIB.btRotationalLimitMotor_getLoLimit,
	ConeTwistConstraint_updateRHS = CLIB.btConeTwistConstraint_updateRHS,
	DiscreteCollisionDetectorInterface_ClosestPointInput_getTransformB = CLIB.btDiscreteCollisionDetectorInterface_ClosestPointInput_getTransformB,
	GImpactQuantizedBvh_buildSet = CLIB.btGImpactQuantizedBvh_buildSet,
	RotationalLimitMotor_getTargetVelocity = CLIB.btRotationalLimitMotor_getTargetVelocity,
	CompoundShape_createAabbTreeFromChildren = CLIB.btCompoundShape_createAabbTreeFromChildren,
	OverlapFilterCallback_needBroadphaseCollision = CLIB.btOverlapFilterCallback_needBroadphaseCollision,
	ConstraintSetting_getDamping = CLIB.btConstraintSetting_getDamping,
	TranslationalLimitMotor2_setMotorCFM = CLIB.btTranslationalLimitMotor2_setMotorCFM,
	Face_delete = CLIB.btFace_delete,
	MultiBody_getBaseForce = CLIB.btMultiBody_getBaseForce,
	ManifoldPoint_getIndex1 = CLIB.btManifoldPoint_getIndex1,
	BroadphaseAabbCallback_process = CLIB.btBroadphaseAabbCallback_process,
	KinematicCharacterController_getGhostObject = CLIB.btKinematicCharacterController_getGhostObject,
	DbvtAabbMm_Extents = CLIB.btDbvtAabbMm_Extents,
	RotationalLimitMotor2_getCurrentLimitError = CLIB.btRotationalLimitMotor2_getCurrentLimitError,
	DbvtAabbMm_FromPoints = CLIB.btDbvtAabbMm_FromPoints,
	HingeConstraint_getLimitSign = CLIB.btHingeConstraint_getLimitSign,
	RaycastVehicle_getWheelTransformWS = CLIB.btRaycastVehicle_getWheelTransformWS,
	UsageBitfield_reset = CLIB.btUsageBitfield_reset,
	MultiBodyLinkCollider_getLink = CLIB.btMultiBodyLinkCollider_getLink,
	AlignedObjectArray_btSoftBody_Link_set = CLIB.btAlignedObjectArray_btSoftBody_Link_set,
	ConvexCast_CastResult_getHitTransformA = CLIB.btConvexCast_CastResult_getHitTransformA,
	AlignedObjectArray_btSoftBody_ClusterPtr_at = CLIB.btAlignedObjectArray_btSoftBody_ClusterPtr_at,
	CollisionWorld_AllHitsRayResultCallback_getHitNormalWorld = CLIB.btCollisionWorld_AllHitsRayResultCallback_getHitNormalWorld,
	AlignedObjectArray_btSoftBody_Link_size = CLIB.btAlignedObjectArray_btSoftBody_Link_size,
	DynamicsWorld_addConstraint2 = CLIB.btDynamicsWorld_addConstraint2,
	TranslationalLimitMotor2_getMotorCFM = CLIB.btTranslationalLimitMotor2_getMotorCFM,
	ConeTwistConstraint_GetPointForAngle = CLIB.btConeTwistConstraint_GetPointForAngle,
	ConvexShape_localGetSupportVertexNonVirtual = CLIB.btConvexShape_localGetSupportVertexNonVirtual,
	GImpactMeshShapePart_TrimeshPrimitiveManager_unlock = CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_unlock,
	ConvexPolyhedron_setRadius = CLIB.btConvexPolyhedron_setRadius,
	WheelInfo_getClientInfo = CLIB.btWheelInfo_getClientInfo,
	CollisionWorld_ClosestConvexResultCallback_setConvexToWorld = CLIB.btCollisionWorld_ClosestConvexResultCallback_setConvexToWorld,
	ConvexPolyhedron_delete = CLIB.btConvexPolyhedron_delete,
	Hinge2Constraint_getAngle1 = CLIB.btHinge2Constraint_getAngle1,
	CompoundShapeChild_getChildMargin = CLIB.btCompoundShapeChild_getChildMargin,
	SortedOverlappingPairCache_needsBroadphaseCollision = CLIB.btSortedOverlappingPairCache_needsBroadphaseCollision,
	CollisionWorld_ClosestRayResultCallback_getHitNormalWorld = CLIB.btCollisionWorld_ClosestRayResultCallback_getHitNormalWorld,
	CylinderShapeZ_new2 = CLIB.btCylinderShapeZ_new2,
	CollisionWorld_ClosestRayResultCallback_getRayFromWorld = CLIB.btCollisionWorld_ClosestRayResultCallback_getRayFromWorld,
	CollisionWorld_ClosestRayResultCallback_getRayToWorld = CLIB.btCollisionWorld_ClosestRayResultCallback_getRayToWorld,
	GImpactShapeInterface_getBulletTetrahedron = CLIB.btGImpactShapeInterface_getBulletTetrahedron,
	SoftBody_indicesToPointers2 = CLIB.btSoftBody_indicesToPointers2,
	CollisionWorld_ContactResultCallback_delete = CLIB.btCollisionWorld_ContactResultCallback_delete,
	DbvtBroadphase_setDeferedcollide = CLIB.btDbvtBroadphase_setDeferedcollide,
	Dbvt_ICollide_Process3 = CLIB.btDbvt_ICollide_Process3,
	CollisionWorld_ConvexResultCallback_getClosestHitFraction = CLIB.btCollisionWorld_ConvexResultCallback_getClosestHitFraction,
	AlignedObjectArray_btPersistentManifoldPtr_resizeNoInitialize = CLIB.btAlignedObjectArray_btPersistentManifoldPtr_resizeNoInitialize,
	CollisionWorld_addCollisionObject3 = CLIB.btCollisionWorld_addCollisionObject3,
	Dbvt_new = CLIB.btDbvt_new,
	DispatcherInfo_setDispatchFunc = CLIB.btDispatcherInfo_setDispatchFunc,
	StorageResult_setDistance = CLIB.btStorageResult_setDistance,
	GImpactMeshShapePart_TrimeshPrimitiveManager_setType = CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_setType,
	GImpactMeshShapePart_TrimeshPrimitiveManager_getType = CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_getType,
	BoxShape_getHalfExtentsWithMargin = CLIB.btBoxShape_getHalfExtentsWithMargin,
	ConvexPlaneCollisionAlgorithm_new = CLIB.btConvexPlaneCollisionAlgorithm_new,
	DiscreteDynamicsWorld_setSynchronizeAllMotionStates = CLIB.btDiscreteDynamicsWorld_setSynchronizeAllMotionStates,
	CollisionWorld_ClosestRayResultCallback_setRayToWorld = CLIB.btCollisionWorld_ClosestRayResultCallback_setRayToWorld,
	CollisionWorld_LocalShapeInfo_getTriangleIndex = CLIB.btCollisionWorld_LocalShapeInfo_getTriangleIndex,
	SoftBody_Cluster_getAv = CLIB.btSoftBody_Cluster_getAv,
	ConeTwistConstraint_getAngularOnly = CLIB.btConeTwistConstraint_getAngularOnly,
	JointFeedback_getAppliedForceBodyB = CLIB.btJointFeedback_getAppliedForceBodyB,
	TranslationalLimitMotor2_setCurrentLinearDiff = CLIB.btTranslationalLimitMotor2_setCurrentLinearDiff,
	CollisionAlgorithm_delete = CLIB.btCollisionAlgorithm_delete,
	CollisionWorld_LocalRayResult_setHitNormalLocal = CLIB.btCollisionWorld_LocalRayResult_setHitNormalLocal,
	CollisionWorld_LocalShapeInfo_setShapePart = CLIB.btCollisionWorld_LocalShapeInfo_setShapePart,
	DbvtAabbMm_FromMM = CLIB.btDbvtAabbMm_FromMM,
	WorldImporter_createPoint2PointConstraint = CLIB.btWorldImporter_createPoint2PointConstraint,
	CollisionWorld_RayResultCallback_getCollisionFilterMask = CLIB.btCollisionWorld_RayResultCallback_getCollisionFilterMask,
	Dispatcher_getNumManifolds = CLIB.btDispatcher_getNumManifolds,
	DiscreteCollisionDetectorInterface_Result_addContactPoint = CLIB.btDiscreteCollisionDetectorInterface_Result_addContactPoint,
	ConeTwistConstraint_setAngularOnly = CLIB.btConeTwistConstraint_setAngularOnly,
	DiscreteCollisionDetectorInterface_Result_setShapeIdentifiersA = CLIB.btDiscreteCollisionDetectorInterface_Result_setShapeIdentifiersA,
	VehicleRaycaster_btVehicleRaycasterResult_getHitNormalInWorld = CLIB.btVehicleRaycaster_btVehicleRaycasterResult_getHitNormalInWorld,
	CollisionAlgorithmConstructionInfo_new2 = CLIB.btCollisionAlgorithmConstructionInfo_new2,
	HingeAccumulatedAngleConstraint_new4 = CLIB.btHingeAccumulatedAngleConstraint_new4,
	StorageResult_getDistance = CLIB.btStorageResult_getDistance,
	BvhTree_getLeftNode = CLIB.btBvhTree_getLeftNode,
	Generic6DofConstraint_getUseLinearReferenceFrameA = CLIB.btGeneric6DofConstraint_getUseLinearReferenceFrameA,
	DbvtAabbMm_Expand = CLIB.btDbvtAabbMm_Expand,
	MultiBody_getAngularMomentum = CLIB.btMultiBody_getAngularMomentum,
	DiscreteDynamicsWorld_new = CLIB.btDiscreteDynamicsWorld_new,
	ConvexTriangleMeshShape_calculatePrincipalAxisTransform = CLIB.btConvexTriangleMeshShape_calculatePrincipalAxisTransform,
	DispatcherInfo_getConvexConservativeDistanceThreshold = CLIB.btDispatcherInfo_getConvexConservativeDistanceThreshold,
	CompoundShapeChild_setChildMargin = CLIB.btCompoundShapeChild_setChildMargin,
	Generic6DofConstraint_isLimited = CLIB.btGeneric6DofConstraint_isLimited,
	DispatcherInfo_getStepCount = CLIB.btDispatcherInfo_getStepCount,
	DispatcherInfo_getTimeStep = CLIB.btDispatcherInfo_getTimeStep,
	DispatcherInfo_setAllowedCcdPenetration = CLIB.btDispatcherInfo_setAllowedCcdPenetration,
	DispatcherInfo_setConvexConservativeDistanceThreshold = CLIB.btDispatcherInfo_setConvexConservativeDistanceThreshold,
	DispatcherInfo_setDebugDraw = CLIB.btDispatcherInfo_setDebugDraw,
	RotationalLimitMotor2_setStopERP = CLIB.btRotationalLimitMotor2_setStopERP,
	MultiBody_setBaseName = CLIB.btMultiBody_setBaseName,
	Generic6DofSpring2Constraint_getFrameOffsetB = CLIB.btGeneric6DofSpring2Constraint_getFrameOffsetB,
	GImpactBvh_getRightNode = CLIB.btGImpactBvh_getRightNode,
	DispatcherInfo_setUseConvexConservativeDistanceUtil = CLIB.btDispatcherInfo_setUseConvexConservativeDistanceUtil,
	TriangleMesh_setWeldingThreshold = CLIB.btTriangleMesh_setWeldingThreshold,
	Dispatcher_clearManifold = CLIB.btDispatcher_clearManifold,
	ConvexConvexAlgorithm_CreateFunc_setNumPerturbationIterations = CLIB.btConvexConvexAlgorithm_CreateFunc_setNumPerturbationIterations,
	MLCPSolver_new = CLIB.btMLCPSolver_new,
	SoftBody_AJoint_IControlWrapper_getWrapperData = CLIB.btSoftBody_AJoint_IControlWrapper_getWrapperData,
	BvhTriangleMeshShape_partialRefitTree = CLIB.btBvhTriangleMeshShape_partialRefitTree,
	Dbvt_update6 = CLIB.btDbvt_update6,
	Dbvt_getRoot = CLIB.btDbvt_getRoot,
	DbvtBroadphase_setReleasepaircache = CLIB.btDbvtBroadphase_setReleasepaircache,
	SoftBody_Impulse_getVelocity = CLIB.btSoftBody_Impulse_getVelocity,
	DynamicsWorld_getConstraint = CLIB.btDynamicsWorld_getConstraint,
	DynamicsWorld_getConstraintSolver = CLIB.btDynamicsWorld_getConstraintSolver,
	BulletXmlWorldImporter_new = CLIB.btBulletXmlWorldImporter_new,
	GImpactShapeInterface_getBoxSet = CLIB.btGImpactShapeInterface_getBoxSet,
	ConeShape_setRadius = CLIB.btConeShape_setRadius,
	DynamicsWorld_setGravity = CLIB.btDynamicsWorld_setGravity,
	DynamicsWorld_stepSimulation = CLIB.btDynamicsWorld_stepSimulation,
	Dbvt_optimizeTopDown2 = CLIB.btDbvt_optimizeTopDown2,
	SliderConstraint_getFrameOffsetB = CLIB.btSliderConstraint_getFrameOffsetB,
	CollisionWorld_performDiscreteCollisionDetection = CLIB.btCollisionWorld_performDiscreteCollisionDetection,
	RotationalLimitMotor2_setCurrentLimitError = CLIB.btRotationalLimitMotor2_setCurrentLimitError,
	HingeConstraint_getBFrame = CLIB.btHingeConstraint_getBFrame,
	DbvtAabbMm_Mins = CLIB.btDbvtAabbMm_Mins,
	CollisionObject_setCompanionId = CLIB.btCollisionObject_setCompanionId,
	MultiBodyConstraint_createConstraintRows = CLIB.btMultiBodyConstraint_createConstraintRows,
	ConvexShape_localGetSupportingVertex = CLIB.btConvexShape_localGetSupportingVertex,
	GhostObject_removeOverlappingObjectInternal2 = CLIB.btGhostObject_removeOverlappingObjectInternal2,
	GImpactMeshShapePart_TrimeshPrimitiveManager_get_bullet_triangle = CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_get_bullet_triangle,
	GearConstraint_setAxisB = CLIB.btGearConstraint_setAxisB,
	RaycastVehicle_applyEngineForce = CLIB.btRaycastVehicle_applyEngineForce,
	RotationalLimitMotor_new2 = CLIB.btRotationalLimitMotor_new2,
	SphereShape_getRadius = CLIB.btSphereShape_getRadius,
	RotationalLimitMotor_setTargetVelocity = CLIB.btRotationalLimitMotor_setTargetVelocity,
	RotationalLimitMotor_getCurrentLimit = CLIB.btRotationalLimitMotor_getCurrentLimit,
	MinkowskiSumShape_getShapeB = CLIB.btMinkowskiSumShape_getShapeB,
	BroadphaseProxy_getUid = CLIB.btBroadphaseProxy_getUid,
	RotationalLimitMotor_getEnableMotor = CLIB.btRotationalLimitMotor_getEnableMotor,
	CollisionWorld_LocalRayResult_setLocalShapeInfo = CLIB.btCollisionWorld_LocalRayResult_setLocalShapeInfo,
	RotationalLimitMotor2_getMaxMotorForce = CLIB.btRotationalLimitMotor2_getMaxMotorForce,
	RotationalLimitMotor_needApplyTorques = CLIB.btRotationalLimitMotor_needApplyTorques,
	RotationalLimitMotor_solveAngularLimits = CLIB.btRotationalLimitMotor_solveAngularLimits,
	CollisionObject_setUserPointer = CLIB.btCollisionObject_setUserPointer,
	RotationalLimitMotor_getMaxMotorForce = CLIB.btRotationalLimitMotor_getMaxMotorForce,
	RotationalLimitMotor_setLoLimit = CLIB.btRotationalLimitMotor_setLoLimit,
	GImpactCompoundShape_CompoundPrimitiveManager_setCompoundShape = CLIB.btGImpactCompoundShape_CompoundPrimitiveManager_setCompoundShape,
	MultibodyLink_setInertiaLocal = CLIB.btMultibodyLink_setInertiaLocal,
	CollisionWorld_RayResultCallback_addSingleResult = CLIB.btCollisionWorld_RayResultCallback_addSingleResult,
	ContactSolverInfoData_getErp = CLIB.btContactSolverInfoData_getErp,
	MinkowskiSumShape_GetTransformB = CLIB.btMinkowskiSumShape_GetTransformB,
	HingeAccumulatedAngleConstraint_new5 = CLIB.btHingeAccumulatedAngleConstraint_new5,
	GImpactQuantizedBvh_new2 = CLIB.btGImpactQuantizedBvh_new2,
	RotationalLimitMotor_setLimitSoftness = CLIB.btRotationalLimitMotor_setLimitSoftness,
	ConvexCast_CastResult_getFraction = CLIB.btConvexCast_CastResult_getFraction,
	BroadphaseProxy_setMultiSapParentProxy = CLIB.btBroadphaseProxy_setMultiSapParentProxy,
	RotationalLimitMotor_setNormalCFM = CLIB.btRotationalLimitMotor_setNormalCFM,
	RotationalLimitMotor_setStopCFM = CLIB.btRotationalLimitMotor_setStopCFM,
	StorageResult_getClosestPointInB = CLIB.btStorageResult_getClosestPointInB,
	TranslationalLimitMotor_getCurrentLimit = CLIB.btTranslationalLimitMotor_getCurrentLimit,
	MultiBodyDynamicsWorld_getNumMultibodies = CLIB.btMultiBodyDynamicsWorld_getNumMultibodies,
	Convex2dConvex2dAlgorithm_CreateFunc_new = CLIB.btConvex2dConvex2dAlgorithm_CreateFunc_new,
	WheelInfoConstructionInfo_getWheelsDampingCompression = CLIB.btWheelInfoConstructionInfo_getWheelsDampingCompression,
	TranslationalLimitMotor_getLimitSoftness = CLIB.btTranslationalLimitMotor_getLimitSoftness,
	GImpactCompoundShape_CompoundPrimitiveManager_new2 = CLIB.btGImpactCompoundShape_CompoundPrimitiveManager_new2,
	TranslationalLimitMotor_getTargetVelocity = CLIB.btTranslationalLimitMotor_getTargetVelocity,
	MultibodyLink_getCollider = CLIB.btMultibodyLink_getCollider,
	TranslationalLimitMotor_setAccumulatedImpulse = CLIB.btTranslationalLimitMotor_setAccumulatedImpulse,
	TriangleBuffer_clearBuffer = CLIB.btTriangleBuffer_clearBuffer,
	TranslationalLimitMotor_setDamping = CLIB.btTranslationalLimitMotor_setDamping,
	MultiBody_getBaseMass = CLIB.btMultiBody_getBaseMass,
	MinkowskiSumShape_new = CLIB.btMinkowskiSumShape_new,
	CollisionWorld_getForceUpdateAllAabbs = CLIB.btCollisionWorld_getForceUpdateAllAabbs,
	MultiBodySolverConstraint_getDeltaVelAindex = CLIB.btMultiBodySolverConstraint_getDeltaVelAindex,
	TranslationalLimitMotor_setMaxMotorForce = CLIB.btTranslationalLimitMotor_setMaxMotorForce,
	PersistentManifold_refreshContactPoints = CLIB.btPersistentManifold_refreshContactPoints,
	DbvtAabbMm_ProjectMinimum = CLIB.btDbvtAabbMm_ProjectMinimum,
	HingeConstraint_new4 = CLIB.btHingeConstraint_new4,
	TranslationalLimitMotor_setTargetVelocity = CLIB.btTranslationalLimitMotor_setTargetVelocity,
	MultiBody_isUsingRK4Integration = CLIB.btMultiBody_isUsingRK4Integration,
	TranslationalLimitMotor_setUpperLimit = CLIB.btTranslationalLimitMotor_setUpperLimit,
	CollisionWorld_LocalConvexResult_getHitPointLocal = CLIB.btCollisionWorld_LocalConvexResult_getHitPointLocal,
	TypedConstraint_internalGetAppliedImpulse = CLIB.btTypedConstraint_internalGetAppliedImpulse,
	SoftBodyRigidBodyCollisionConfiguration_new2 = CLIB.btSoftBodyRigidBodyCollisionConfiguration_new2,
	Generic6DofSpring2Constraint_setBounce = CLIB.btGeneric6DofSpring2Constraint_setBounce,
	Generic6DofSpringConstraint_setEquilibriumPoint3 = CLIB.btGeneric6DofSpringConstraint_setEquilibriumPoint3,
	CollisionWorld_RayResultCallbackWrapper_needsCollision = CLIB.btCollisionWorld_RayResultCallbackWrapper_needsCollision,
	HingeConstraint_new7 = CLIB.btHingeConstraint_new7,
	MultiBodyLinkCollider_upcast = CLIB.btMultiBodyLinkCollider_upcast,
	GImpactMeshShapePart_TrimeshPrimitiveManager_new2 = CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_new2,
	PairCachingGhostObject_getOverlappingPairCache = CLIB.btPairCachingGhostObject_getOverlappingPairCache,
	TriangleMesh_addTriangle = CLIB.btTriangleMesh_addTriangle,
	SubSimplexClosestResult_getDegenerate = CLIB.btSubSimplexClosestResult_getDegenerate,
	Generic6DofConstraint_getUseFrameOffset = CLIB.btGeneric6DofConstraint_getUseFrameOffset,
	CollisionShape_setUserIndex = CLIB.btCollisionShape_setUserIndex,
	DynamicsWorld_removeRigidBody = CLIB.btDynamicsWorld_removeRigidBody,
	WorldImporter_createConeTwistConstraint = CLIB.btWorldImporter_createConeTwistConstraint,
	Generic6DofConstraint_setLinearLowerLimit = CLIB.btGeneric6DofConstraint_setLinearLowerLimit,
	Generic6DofConstraint_setLinearUpperLimit = CLIB.btGeneric6DofConstraint_setLinearUpperLimit,
	GImpactBvh_find_collision = CLIB.btGImpactBvh_find_collision,
	Dbvt_sStkCLN_new = CLIB.btDbvt_sStkCLN_new,
	UniversalConstraint_getAngle2 = CLIB.btUniversalConstraint_getAngle2,
	DynamicsWorld_stepSimulation3 = CLIB.btDynamicsWorld_stepSimulation3,
	VoronoiSimplexSolver_closest = CLIB.btVoronoiSimplexSolver_closest,
	RotationalLimitMotor2_getLoLimit = CLIB.btRotationalLimitMotor2_getLoLimit,
	Dbvt_getRayTestStack = CLIB.btDbvt_getRayTestStack,
	PersistentManifold_addManifoldPoint = CLIB.btPersistentManifold_addManifoldPoint,
	CompoundShapeChild_getChildShape = CLIB.btCompoundShapeChild_getChildShape,
	CollisionDispatcher_setCollisionConfiguration = CLIB.btCollisionDispatcher_setCollisionConfiguration,
	RotationalLimitMotor2_isLimited = CLIB.btRotationalLimitMotor2_isLimited,
	RotationalLimitMotor2_setCurrentLimit = CLIB.btRotationalLimitMotor2_setCurrentLimit,
	RotationalLimitMotor2_setCurrentLimitErrorHi = CLIB.btRotationalLimitMotor2_setCurrentLimitErrorHi,
	RotationalLimitMotor2_setCurrentPosition = CLIB.btRotationalLimitMotor2_setCurrentPosition,
	GImpactMeshShapePart_TrimeshPrimitiveManager_getVertexbase = CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_getVertexbase,
	MultibodyLink_getDofCount = CLIB.btMultibodyLink_getDofCount,
	RotationalLimitMotor2_setEquilibriumPoint = CLIB.btRotationalLimitMotor2_setEquilibriumPoint,
	RotationalLimitMotor2_setLoLimit = CLIB.btRotationalLimitMotor2_setLoLimit,
	MultiBodySolverConstraint_getSolverBodyIdB = CLIB.btMultiBodySolverConstraint_getSolverBodyIdB,
	RotationalLimitMotor2_setServoMotor = CLIB.btRotationalLimitMotor2_setServoMotor,
	CapsuleShapeX_new = CLIB.btCapsuleShapeX_new,
	CollisionWorld_ContactResultCallback_getCollisionFilterGroup = CLIB.btCollisionWorld_ContactResultCallback_getCollisionFilterGroup,
	CollisionWorld_ContactResultCallbackWrapper_needsCollision = CLIB.btCollisionWorld_ContactResultCallbackWrapper_needsCollision,
	GImpactQuantizedBvh_getNodeBound = CLIB.btGImpactQuantizedBvh_getNodeBound,
	BroadphaseRayCallback_setRayDirectionInverse = CLIB.btBroadphaseRayCallback_setRayDirectionInverse,
	DbvtBroadphase_getGid = CLIB.btDbvtBroadphase_getGid,
	MultiBody_setLinearDamping = CLIB.btMultiBody_setLinearDamping,
	TranslationalLimitMotor2_getCurrentLimitError = CLIB.btTranslationalLimitMotor2_getCurrentLimitError,
	CollisionAlgorithm_calculateTimeOfImpact = CLIB.btCollisionAlgorithm_calculateTimeOfImpact,
	Generic6DofSpringConstraint_getEquilibriumPoint = CLIB.btGeneric6DofSpringConstraint_getEquilibriumPoint,
	SoftBody_clusterVelocity = CLIB.btSoftBody_clusterVelocity,
	MultiBody_setAngularDamping = CLIB.btMultiBody_setAngularDamping,
	GImpactQuantizedBvh_isTrimesh = CLIB.btGImpactQuantizedBvh_isTrimesh,
	MultibodyLink_getFlags = CLIB.btMultibodyLink_getFlags,
	SoftBody_Body_angularVelocity = CLIB.btSoftBody_Body_angularVelocity,
	Hinge2Constraint_setUpperLimit = CLIB.btHinge2Constraint_setUpperLimit,
	WorldImporter_createConeShapeX = CLIB.btWorldImporter_createConeShapeX,
	ShapeHull_numTriangles = CLIB.btShapeHull_numTriangles,
	TranslationalLimitMotor2_getSpringStiffnessLimited = CLIB.btTranslationalLimitMotor2_getSpringStiffnessLimited,
	VoronoiSimplexSolver_getNumVertices = CLIB.btVoronoiSimplexSolver_getNumVertices,
	GImpactMeshShapePart_TrimeshPrimitiveManager_setLock_count = CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_setLock_count,
	DbvtBroadphase_setNewpairs = CLIB.btDbvtBroadphase_setNewpairs,
	PersistentManifold_setBodies = CLIB.btPersistentManifold_setBodies,
	KinematicCharacterController_getGravity = CLIB.btKinematicCharacterController_getGravity,
	TranslationalLimitMotor2_setLowerLimit = CLIB.btTranslationalLimitMotor2_setLowerLimit,
	TranslationalLimitMotor2_setMaxMotorForce = CLIB.btTranslationalLimitMotor2_setMaxMotorForce,
	TranslationalLimitMotor2_setMotorERP = CLIB.btTranslationalLimitMotor2_setMotorERP,
	TranslationalLimitMotor2_setSpringDamping = CLIB.btTranslationalLimitMotor2_setSpringDamping,
	TranslationalLimitMotor2_setSpringStiffness = CLIB.btTranslationalLimitMotor2_setSpringStiffness,
	TranslationalLimitMotor2_setStopCFM = CLIB.btTranslationalLimitMotor2_setStopCFM,
	TranslationalLimitMotor2_setTargetVelocity = CLIB.btTranslationalLimitMotor2_setTargetVelocity,
	TranslationalLimitMotor2_setUpperLimit = CLIB.btTranslationalLimitMotor2_setUpperLimit,
	SerializerWrapper_new = CLIB.btSerializerWrapper_new,
	MultiBody_forwardKinematics = CLIB.btMultiBody_forwardKinematics,
	CollisionWorld_debugDrawWorld = CLIB.btCollisionWorld_debugDrawWorld,
	CollisionObject_hasAnisotropicFriction = CLIB.btCollisionObject_hasAnisotropicFriction,
	Generic6DofSpring2Constraint_getAngularUpperLimit = CLIB.btGeneric6DofSpring2Constraint_getAngularUpperLimit,
	BulletXmlWorldImporter_loadFile = CLIB.btBulletXmlWorldImporter_loadFile,
	MultiBody_getBaseInertia = CLIB.btMultiBody_getBaseInertia,
	Generic6DofSpring2Constraint_getCalculatedTransformB = CLIB.btGeneric6DofSpring2Constraint_getCalculatedTransformB,
	Dbvt_sStkNPS_setNode = CLIB.btDbvt_sStkNPS_setNode,
	ManifoldPoint_setDistance1 = CLIB.btManifoldPoint_setDistance1,
	MultiBodyConstraint_setMaxAppliedImpulse = CLIB.btMultiBodyConstraint_setMaxAppliedImpulse,
	SparseSdf_new = CLIB.btSparseSdf_new,
	Generic6DofSpring2Constraint_setEquilibriumPoint = CLIB.btGeneric6DofSpring2Constraint_setEquilibriumPoint,
	Generic6DofSpring2Constraint_setAngularUpperLimitReversed = CLIB.btGeneric6DofSpring2Constraint_setAngularUpperLimitReversed,
	CompoundShape_calculatePrincipalAxisTransform = CLIB.btCompoundShape_calculatePrincipalAxisTransform,
	CollisionShape_getAngularMotionDisc = CLIB.btCollisionShape_getAngularMotionDisc,
	SoftBody_addAeroForceToFace = CLIB.btSoftBody_addAeroForceToFace,
	TranslationalLimitMotor_getStopCFM = CLIB.btTranslationalLimitMotor_getStopCFM,
	BvhTree_get_node_pointer2 = CLIB.btBvhTree_get_node_pointer2,
	CylinderShape_getUpAxis = CLIB.btCylinderShape_getUpAxis,
	RotationalLimitMotor2_setSpringStiffness = CLIB.btRotationalLimitMotor2_setSpringStiffness,
	ConeTwistConstraint_setDamping = CLIB.btConeTwistConstraint_setDamping,
	ConeTwistConstraint_getTwistSpan = CLIB.btConeTwistConstraint_getTwistSpan,
	DefaultCollisionConstructionInfo_setPersistentManifoldPool = CLIB.btDefaultCollisionConstructionInfo_setPersistentManifoldPool,
	Triangle_new = CLIB.btTriangle_new,
	GImpactQuantizedBvh_find_collision = CLIB.btGImpactQuantizedBvh_find_collision,
	CollisionObject_getCcdMotionThreshold = CLIB.btCollisionObject_getCcdMotionThreshold,
	StaticPlaneShape_new = CLIB.btStaticPlaneShape_new,
	Generic6DofSpring2Constraint_new3 = CLIB.btGeneric6DofSpring2Constraint_new3,
	SoftBody_Cluster_getFramerefs = CLIB.btSoftBody_Cluster_getFramerefs,
	ConstraintSetting_delete = CLIB.btConstraintSetting_delete,
	CollisionWorld_ContactResultCallback_setCollisionFilterMask = CLIB.btCollisionWorld_ContactResultCallback_setCollisionFilterMask,
	GImpactCompoundShape_CompoundPrimitiveManager_new = CLIB.btGImpactCompoundShape_CompoundPrimitiveManager_new,
	GjkPairDetector_setCatchDegeneracies = CLIB.btGjkPairDetector_setCatchDegeneracies,
	SoftBody_Node_setX = CLIB.btSoftBody_Node_setX,
	SoftBody_appendNode = CLIB.btSoftBody_appendNode,
	CollisionAlgorithm_getAllContactManifolds = CLIB.btCollisionAlgorithm_getAllContactManifolds,
	TriangleInfoMap_setEqualVertexThreshold = CLIB.btTriangleInfoMap_setEqualVertexThreshold,
	CollisionDispatcher_registerCollisionCreateFunc = CLIB.btCollisionDispatcher_registerCollisionCreateFunc,
	CollisionWorld_LocalRayResult_setCollisionObject = CLIB.btCollisionWorld_LocalRayResult_setCollisionObject,
	SoftBody_Note_getRank = CLIB.btSoftBody_Note_getRank,
	TypedConstraint_getRigidBodyA = CLIB.btTypedConstraint_getRigidBodyA,
	MultiBody_getMaxCoordinateVelocity = CLIB.btMultiBody_getMaxCoordinateVelocity,
	MultiBody_setupSpherical2 = CLIB.btMultiBody_setupSpherical2,
	Chunk_setLength = CLIB.btChunk_setLength,
	TranslationalLimitMotor_setCurrentLinearDiff = CLIB.btTranslationalLimitMotor_setCurrentLinearDiff,
	RigidBody_computeGyroscopicForceExplicit = CLIB.btRigidBody_computeGyroscopicForceExplicit,
	Generic6DofSpringConstraint_setStiffness = CLIB.btGeneric6DofSpringConstraint_setStiffness,
	Generic6DofConstraint_setAngularUpperLimit = CLIB.btGeneric6DofConstraint_setAngularUpperLimit,
	WheelInfo_setSkidInfo = CLIB.btWheelInfo_setSkidInfo,
	CollisionWorld_rayTestSingleInternal = CLIB.btCollisionWorld_rayTestSingleInternal,
	AABB_overlapping_trans_conservative2 = CLIB.btAABB_overlapping_trans_conservative2,
	MultiBodyConstraint_getPosition = CLIB.btMultiBodyConstraint_getPosition,
	GImpactShapeInterface_getChildShape = CLIB.btGImpactShapeInterface_getChildShape,
	DefaultSerializer_new2 = CLIB.btDefaultSerializer_new2,
	DiscreteDynamicsWorld_getLatencyMotionStateInterpolation = CLIB.btDiscreteDynamicsWorld_getLatencyMotionStateInterpolation,
	BroadphasePair_setPProxy1 = CLIB.btBroadphasePair_setPProxy1,
	WorldImporter_createGeneric6DofConstraint2 = CLIB.btWorldImporter_createGeneric6DofConstraint2,
	GImpactShapeInterface_childrenHasTransform = CLIB.btGImpactShapeInterface_childrenHasTransform,
	ManifoldPoint_setContactERP = CLIB.btManifoldPoint_setContactERP,
	DefaultCollisionConstructionInfo_setCustomCollisionAlgorithmMaxElementSize = CLIB.btDefaultCollisionConstructionInfo_setCustomCollisionAlgorithmMaxElementSize,
	Dbvt_sStkNPS_getMask = CLIB.btDbvt_sStkNPS_getMask,
	Generic6DofConstraint_testAngularLimitMotor = CLIB.btGeneric6DofConstraint_testAngularLimitMotor,
	RotationalLimitMotor2_getSpringDampingLimited = CLIB.btRotationalLimitMotor2_getSpringDampingLimited,
	AABB_new = CLIB.btAABB_new,
	DbvtBroadphase_getUpdates_ratio = CLIB.btDbvtBroadphase_getUpdates_ratio,
	TransformUtil_calculateVelocity = CLIB.btTransformUtil_calculateVelocity,
	GImpactBvh_getNodeBound = CLIB.btGImpactBvh_getNodeBound,
	Generic6DofConstraint_new = CLIB.btGeneric6DofConstraint_new,
	CapsuleShape_new = CLIB.btCapsuleShape_new,
	GImpactQuantizedBvh_getPrimitiveManager = CLIB.btGImpactQuantizedBvh_getPrimitiveManager,
	DefaultMotionState_setStartWorldTrans = CLIB.btDefaultMotionState_setStartWorldTrans,
	TranslationalLimitMotor2_getUpperLimit = CLIB.btTranslationalLimitMotor2_getUpperLimit,
	HingeAccumulatedAngleConstraint_new3 = CLIB.btHingeAccumulatedAngleConstraint_new3,
	GImpactCollisionAlgorithm_new = CLIB.btGImpactCollisionAlgorithm_new,
	CollisionObject_setCollisionFlags = CLIB.btCollisionObject_setCollisionFlags,
	SoftBody_RContact_new = CLIB.btSoftBody_RContact_new,
	DiscreteDynamicsWorld_updateVehicles = CLIB.btDiscreteDynamicsWorld_updateVehicles,
	SoftBody_checkContact = CLIB.btSoftBody_checkContact,
	TriangleMeshShape_localGetSupportingVertex = CLIB.btTriangleMeshShape_localGetSupportingVertex,
	GImpactCollisionAlgorithm_setPart1 = CLIB.btGImpactCollisionAlgorithm_setPart1,
	GjkPairDetector_setLastUsedMethod = CLIB.btGjkPairDetector_setLastUsedMethod,
	AABB_get_center_extend = CLIB.btAABB_get_center_extend,
	ConstraintSolver_allSolved = CLIB.btConstraintSolver_allSolved,
	Dbvt_rayTestInternal = CLIB.btDbvt_rayTestInternal,
	DbvtBroadphase_getCupdates = CLIB.btDbvtBroadphase_getCupdates,
	TranslationalLimitMotor2_getServoTarget = CLIB.btTranslationalLimitMotor2_getServoTarget,
	ManifoldPoint_getLateralFrictionDir2 = CLIB.btManifoldPoint_getLateralFrictionDir2,
	AngularLimit_set = CLIB.btAngularLimit_set,
	ManifoldPoint_setNormalWorldOnB = CLIB.btManifoldPoint_setNormalWorldOnB,
	MultibodyLink_setAppliedTorque = CLIB.btMultibodyLink_setAppliedTorque,
	QuantizedBvhTree_getEscapeNodeIndex = CLIB.btQuantizedBvhTree_getEscapeNodeIndex,
	GImpactQuantizedBvh_boxQuery = CLIB.btGImpactQuantizedBvh_boxQuery,
	SimulationIslandManager_updateActivationState = CLIB.btSimulationIslandManager_updateActivationState,
	Generic6DofConstraint_getFrameOffsetA = CLIB.btGeneric6DofConstraint_getFrameOffsetA,
	RotationalLimitMotor2_setTargetVelocity = CLIB.btRotationalLimitMotor2_setTargetVelocity,
	GjkPairDetector_setMinkowskiA = CLIB.btGjkPairDetector_setMinkowskiA,
	DbvtBroadphase_setCupdates = CLIB.btDbvtBroadphase_setCupdates,
	ConeTwistConstraint_getDamping = CLIB.btConeTwistConstraint_getDamping,
	MultiBodyConstraintSolver_solveGroupCacheFriendlyFinish = CLIB.btMultiBodyConstraintSolver_solveGroupCacheFriendlyFinish,
	Generic6DofSpringConstraint_new = CLIB.btGeneric6DofSpringConstraint_new,
	ConeTwistConstraint_getSwingSpan2 = CLIB.btConeTwistConstraint_getSwingSpan2,
	GImpactQuantizedBvh_update = CLIB.btGImpactQuantizedBvh_update,
	DynamicsWorld_removeConstraint = CLIB.btDynamicsWorld_removeConstraint,
	ConvexCast_calcTimeOfImpact = CLIB.btConvexCast_calcTimeOfImpact,
	Generic6DofSpring2Constraint_setTargetVelocity = CLIB.btGeneric6DofSpring2Constraint_setTargetVelocity,
	ConvexConcaveCollisionAlgorithm_SwappedCreateFunc_new = CLIB.btConvexConcaveCollisionAlgorithm_SwappedCreateFunc_new,
	RotationalLimitMotor2_getStopERP = CLIB.btRotationalLimitMotor2_getStopERP,
	GImpactBvh_new = CLIB.btGImpactBvh_new,
	Dbvt_setFree = CLIB.btDbvt_setFree,
	MultibodyLink_getJointDamping = CLIB.btMultibodyLink_getJointDamping,
	RaycastVehicle_resetSuspension = CLIB.btRaycastVehicle_resetSuspension,
	GImpactShapeInterface_unlockChildShapes = CLIB.btGImpactShapeInterface_unlockChildShapes,
	GImpactCompoundShape_CompoundPrimitiveManager_getCompoundShape = CLIB.btGImpactCompoundShape_CompoundPrimitiveManager_getCompoundShape,
	CollisionWorld_getBroadphase = CLIB.btCollisionWorld_getBroadphase,
	SoftBody_new2 = CLIB.btSoftBody_new2,
	GImpactMeshShapePart_TrimeshPrimitiveManager_new = CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_new,
	RotationalLimitMotor_getDamping = CLIB.btRotationalLimitMotor_getDamping,
	SoftBody_SContact_getMargin = CLIB.btSoftBody_SContact_getMargin,
	GImpactMeshShapePart_TrimeshPrimitiveManager_getNumverts = CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_getNumverts,
	BvhTriangleMeshShape_serializeSingleBvh = CLIB.btBvhTriangleMeshShape_serializeSingleBvh,
	DbvtNode_getData = CLIB.btDbvtNode_getData,
	StorageResult_setClosestPointInB = CLIB.btStorageResult_setClosestPointInB,
	AxisSweep3_removeHandle = CLIB.btAxisSweep3_removeHandle,
	GImpactMeshShapePart_TrimeshPrimitiveManager_setIndexstride = CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_setIndexstride,
	SoftBody_appendTetra2 = CLIB.btSoftBody_appendTetra2,
	GImpactMeshShapePart_TrimeshPrimitiveManager_get_vertex = CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_get_vertex,
	MultiBody_addJointTorque = CLIB.btMultiBody_addJointTorque,
	Hinge2Constraint_getAnchor2 = CLIB.btHinge2Constraint_getAnchor2,
	GImpactMeshShapePart_TrimeshPrimitiveManager_setScale = CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_setScale,
	ConvexPlaneCollisionAlgorithm_CreateFunc_setMinimumPointsPerturbationThreshold = CLIB.btConvexPlaneCollisionAlgorithm_CreateFunc_setMinimumPointsPerturbationThreshold,
	CollisionWorld_RayResultCallback_getFlags = CLIB.btCollisionWorld_RayResultCallback_getFlags,
	ConvexTriangleCallback_getTriangleCount = CLIB.btConvexTriangleCallback_getTriangleCount,
	AlignedObjectArray_btSoftBodyPtr_size = CLIB.btAlignedObjectArray_btSoftBodyPtr_size,
	ConvexInternalShape_setSafeMargin4 = CLIB.btConvexInternalShape_setSafeMargin4,
	RotationalLimitMotor_setHiLimit = CLIB.btRotationalLimitMotor_setHiLimit,
	RotationalLimitMotor_setDamping = CLIB.btRotationalLimitMotor_setDamping,
	Generic6DofSpring2Constraint_setLinearLowerLimit = CLIB.btGeneric6DofSpring2Constraint_setLinearLowerLimit,
	Generic6DofSpring2Constraint_setAngularLowerLimit = CLIB.btGeneric6DofSpring2Constraint_setAngularLowerLimit,
	Dbvt_countLeaves = CLIB.btDbvt_countLeaves,
	CollisionDispatcher_new = CLIB.btCollisionDispatcher_new,
	InternalTriangleIndexCallback_delete = CLIB.btInternalTriangleIndexCallback_delete,
	MultiBody_setJointVel = CLIB.btMultiBody_setJointVel,
	WorldImporter_setVerboseMode = CLIB.btWorldImporter_setVerboseMode,
	GjkPairDetector_setCurIter = CLIB.btGjkPairDetector_setCurIter,
	GImpactQuantizedBvh_getNodeData = CLIB.btGImpactQuantizedBvh_getNodeData,
	BroadphaseRayCallback_getLambda_max = CLIB.btBroadphaseRayCallback_getLambda_max,
	Generic6DofSpring2Constraint_getAngularLowerLimit = CLIB.btGeneric6DofSpring2Constraint_getAngularLowerLimit,
	HeightfieldTerrainShape_new = CLIB.btHeightfieldTerrainShape_new,
	RigidBody_btRigidBodyConstructionInfo_getAdditionalLinearDampingThresholdSqr = CLIB.btRigidBody_btRigidBodyConstructionInfo_getAdditionalLinearDampingThresholdSqr,
	DispatcherInfo_setStepCount = CLIB.btDispatcherInfo_setStepCount,
	CollisionObject_setHitFraction = CLIB.btCollisionObject_setHitFraction,
	ContactSolverInfoData_setRestitution = CLIB.btContactSolverInfoData_setRestitution,
	MultibodyLink_getAxisBottom = CLIB.btMultibodyLink_getAxisBottom,
	HingeConstraint_new2 = CLIB.btHingeConstraint_new2,
	MultibodyLink_getJointFeedback = CLIB.btMultibodyLink_getJointFeedback,
	GImpactShapeInterface_rayTest = CLIB.btGImpactShapeInterface_rayTest,
	DefaultCollisionConfiguration_setConvexConvexMultipointIterations3 = CLIB.btDefaultCollisionConfiguration_setConvexConvexMultipointIterations3,
	ManifoldResult_getBody1Internal = CLIB.btManifoldResult_getBody1Internal,
	MultibodyLink_getMass = CLIB.btMultibodyLink_getMass,
	HingeConstraint_enableMotor = CLIB.btHingeConstraint_enableMotor,
	EmptyAlgorithm_CreateFunc_new = CLIB.btEmptyAlgorithm_CreateFunc_new,
	HingeConstraint_getFrameOffsetA = CLIB.btHingeConstraint_getFrameOffsetA,
	HingeConstraint_getHingeAngle2 = CLIB.btHingeConstraint_getHingeAngle2,
	OverlappingPairCache_getOverlappingPairArrayPtr = CLIB.btOverlappingPairCache_getOverlappingPairArrayPtr,
	MultiBody_getParentToLocalRot = CLIB.btMultiBody_getParentToLocalRot,
	BvhTree_getNodeData = CLIB.btBvhTree_getNodeData,
	KinematicCharacterController_new = CLIB.btKinematicCharacterController_new,
	BroadphaseProxy_isConvex2d = CLIB.btBroadphaseProxy_isConvex2d,
	MultibodyLink_getAppliedConstraintTorque = CLIB.btMultibodyLink_getAppliedConstraintTorque,
	TranslationalLimitMotor_needApplyForce = CLIB.btTranslationalLimitMotor_needApplyForce,
	HingeConstraint_getAFrame = CLIB.btHingeConstraint_getAFrame,
	ConvexTriangleMeshShape_new2 = CLIB.btConvexTriangleMeshShape_new2,
	IndexedMesh_getVertexType = CLIB.btIndexedMesh_getVertexType,
	ConeTwistConstraint_getTwistLimitSign = CLIB.btConeTwistConstraint_getTwistLimitSign,
	HingeConstraint_setAngularOnly = CLIB.btHingeConstraint_setAngularOnly,
	SparseSdf_delete = CLIB.btSparseSdf_delete,
	SoftBody_Config_getAeromodel = CLIB.btSoftBody_Config_getAeromodel,
	JointFeedback_delete = CLIB.btJointFeedback_delete,
	GjkPairDetector_setIgnoreMargin = CLIB.btGjkPairDetector_setIgnoreMargin,
	RotationalLimitMotor_setCurrentLimitError = CLIB.btRotationalLimitMotor_setCurrentLimitError,
	SoftBody_Joint_getMassmatrix = CLIB.btSoftBody_Joint_getMassmatrix,
	MultiBodySolverConstraint_getRelpos1CrossNormal = CLIB.btMultiBodySolverConstraint_getRelpos1CrossNormal,
	CollisionObject_getInternalType = CLIB.btCollisionObject_getInternalType,
	Convex2dConvex2dAlgorithm_getManifold = CLIB.btConvex2dConvex2dAlgorithm_getManifold,
	Serializer_delete = CLIB.btSerializer_delete,
	BulletWorldImporter_new = CLIB.btBulletWorldImporter_new,
	QuantizedBvhTree_quantizePoint = CLIB.btQuantizedBvhTree_quantizePoint,
	TranslationalLimitMotor2_setEquilibriumPoint = CLIB.btTranslationalLimitMotor2_setEquilibriumPoint,
	OptimizedBvhNode_getAabbMaxOrg = CLIB.btOptimizedBvhNode_getAabbMaxOrg,
	KinematicCharacterController_setFallSpeed = CLIB.btKinematicCharacterController_setFallSpeed,
	MultiBody_applyDeltaVeeMultiDof2 = CLIB.btMultiBody_applyDeltaVeeMultiDof2,
	SparseSdf3_GarbageCollect2 = CLIB.btSparseSdf3_GarbageCollect2,
	ConvexTriangleMeshShape_getMeshInterface = CLIB.btConvexTriangleMeshShape_getMeshInterface,
	AngularLimit_getLow = CLIB.btAngularLimit_getLow,
	RigidBody_getCenterOfMassTransform = CLIB.btRigidBody_getCenterOfMassTransform,
	VoronoiSimplexSolver_delete = CLIB.btVoronoiSimplexSolver_delete,
	CollisionDispatcher_getCollisionConfiguration = CLIB.btCollisionDispatcher_getCollisionConfiguration,
	Dispatcher_findAlgorithm = CLIB.btDispatcher_findAlgorithm,
	ManifoldPoint_getAppliedImpulseLateral2 = CLIB.btManifoldPoint_getAppliedImpulseLateral2,
	DbvtBroadphase_setPaircache = CLIB.btDbvtBroadphase_setPaircache,
	ManifoldPoint_getCombinedRestitution = CLIB.btManifoldPoint_getCombinedRestitution,
	ManifoldPoint_getContactCFM = CLIB.btManifoldPoint_getContactCFM,
	ManifoldPoint_getContactERP = CLIB.btManifoldPoint_getContactERP,
	ManifoldPoint_getDistance = CLIB.btManifoldPoint_getDistance,
	ManifoldPoint_getLateralFrictionDir1 = CLIB.btManifoldPoint_getLateralFrictionDir1,
	SoftBody_getCdbvt = CLIB.btSoftBody_getCdbvt,
	CollisionWorld_ClosestConvexResultCallback_new = CLIB.btCollisionWorld_ClosestConvexResultCallback_new,
	MultibodyLink_setAxisTop2 = CLIB.btMultibodyLink_setAxisTop2,
	CollisionObject_setAnisotropicFriction2 = CLIB.btCollisionObject_setAnisotropicFriction2,
	CollisionShape_getShapeType = CLIB.btCollisionShape_getShapeType,
	QuantizedBvh_setQuantizationValues = CLIB.btQuantizedBvh_setQuantizationValues,
	DbvtBroadphase_setFixedleft = CLIB.btDbvtBroadphase_setFixedleft,
	ConvexTriangleCallback_setTimeStepAndCounters = CLIB.btConvexTriangleCallback_setTimeStepAndCounters,
	AABB_collide_plane = CLIB.btAABB_collide_plane,
	CompoundShapeChild_setChildShapeType = CLIB.btCompoundShapeChild_setChildShapeType,
	ManifoldPoint_setIndex0 = CLIB.btManifoldPoint_setIndex0,
	ManifoldPoint_setLateralFrictionDir1 = CLIB.btManifoldPoint_setLateralFrictionDir1,
	CompoundFromGImpact_btCreateCompoundFromGimpactShape = CLIB.btCompoundFromGImpact_btCreateCompoundFromGimpactShape,
	Dbvt_getStkStack = CLIB.btDbvt_getStkStack,
	SoftBody_Cluster_getSelfCollisionImpulseFactor = CLIB.btSoftBody_Cluster_getSelfCollisionImpulseFactor,
	SoftBodyWorldInfo_new = CLIB.btSoftBodyWorldInfo_new,
	WorldImporter_createCapsuleShapeZ = CLIB.btWorldImporter_createCapsuleShapeZ,
	ManifoldPoint_setCombinedRollingFriction = CLIB.btManifoldPoint_setCombinedRollingFriction,
	MultiBody_clearVelocities = CLIB.btMultiBody_clearVelocities,
	ManifoldResult_getBody1Wrap = CLIB.btManifoldResult_getBody1Wrap,
	AABB_collide_triangle_exact = CLIB.btAABB_collide_triangle_exact,
	CollisionWorld_LocalConvexResult_setHitFraction = CLIB.btCollisionWorld_LocalConvexResult_setHitFraction,
	UnionFind_allocate = CLIB.btUnionFind_allocate,
	TranslationalLimitMotor2_isLimited = CLIB.btTranslationalLimitMotor2_isLimited,
	RotationalLimitMotor_setBounce = CLIB.btRotationalLimitMotor_setBounce,
	WheelInfo_getSuspensionRestLength1 = CLIB.btWheelInfo_getSuspensionRestLength1,
	SoftBody_appendFace5 = CLIB.btSoftBody_appendFace5,
	SoftBody_Impulse_operator_n = CLIB.btSoftBody_Impulse_operator_n,
	TranslationalLimitMotor2_new = CLIB.btTranslationalLimitMotor2_new,
	ManifoldResult_setBody0Wrap = CLIB.btManifoldResult_setBody0Wrap,
	MultiBody_addBaseForce = CLIB.btMultiBody_addBaseForce,
	MultiBody_addBaseTorque = CLIB.btMultiBody_addBaseTorque,
	ContactSolverInfoData_setRestingContactRestitutionThreshold = CLIB.btContactSolverInfoData_setRestingContactRestitutionThreshold,
	GImpactBvh_boxQuery = CLIB.btGImpactBvh_boxQuery,
	RigidBody_btRigidBodyConstructionInfo_new2 = CLIB.btRigidBody_btRigidBodyConstructionInfo_new2,
	MultiBody_addLinkForce = CLIB.btMultiBody_addLinkForce,
	AxisSweep3_getNumHandles = CLIB.btAxisSweep3_getNumHandles,
	ContactSolverInfoData_getNumIterations = CLIB.btContactSolverInfoData_getNumIterations,
	Generic6DofSpring2Constraint_getCalculatedTransformA = CLIB.btGeneric6DofSpring2Constraint_getCalculatedTransformA,
	TranslationalLimitMotor_setLimitSoftness = CLIB.btTranslationalLimitMotor_setLimitSoftness,
	WheelInfo_getWheelsSuspensionForce = CLIB.btWheelInfo_getWheelsSuspensionForce,
	ConvexPolyhedron_project = CLIB.btConvexPolyhedron_project,
	MultiBody_getBaseWorldTransform = CLIB.btMultiBody_getBaseWorldTransform,
	AxisSweep3_new = CLIB.btAxisSweep3_new,
	MultiBody_getJointPosMultiDof = CLIB.btMultiBody_getJointPosMultiDof,
	MultiBody_goToSleep = CLIB.btMultiBody_goToSleep,
	Generic6DofSpring2Constraint_getLinearLowerLimit = CLIB.btGeneric6DofSpring2Constraint_getLinearLowerLimit,
	BulletWorldImporter_loadFileFromMemory2 = CLIB.btBulletWorldImporter_loadFileFromMemory2,
	MultiBody_getLinkMass = CLIB.btMultiBody_getLinkMass,
	MultiBodyConstraint_getNumRows = CLIB.btMultiBodyConstraint_getNumRows,
	Dbvt_sStkNP_getMask = CLIB.btDbvt_sStkNP_getMask,
	GImpactMeshShapePart_TrimeshPrimitiveManager_setMargin = CLIB.btGImpactMeshShapePart_TrimeshPrimitiveManager_setMargin,
	MultiBody_localDirToWorld = CLIB.btMultiBody_localDirToWorld,
	ConvexTriangleCallback_clearWrapperData = CLIB.btConvexTriangleCallback_clearWrapperData,
	MultiBody_setJointPosMultiDof = CLIB.btMultiBody_setJointPosMultiDof,
	IndexedMesh_setVertexBase = CLIB.btIndexedMesh_setVertexBase,
	MultibodyLink_getEVector = CLIB.btMultibodyLink_getEVector,
	MultiBody_setupPlanar = CLIB.btMultiBody_setupPlanar,
	MultiBody_setupRevolute2 = CLIB.btMultiBody_setupRevolute2,
	CompoundShape_new3 = CLIB.btCompoundShape_new3,
	WheelInfo_setBIsFrontWheel = CLIB.btWheelInfo_setBIsFrontWheel,
	AlignedObjectArray_btCollisionObjectPtr_resizeNoInitialize = CLIB.btAlignedObjectArray_btCollisionObjectPtr_resizeNoInitialize,
	SphereShape_setUnscaledRadius = CLIB.btSphereShape_setUnscaledRadius,
	MultiBody_getNumLinks = CLIB.btMultiBody_getNumLinks,
	Point2PointConstraint_setPivotB = CLIB.btPoint2PointConstraint_setPivotB,
	Generic6DofConstraint_calculateTransforms = CLIB.btGeneric6DofConstraint_calculateTransforms,
	MultibodyLink_getAppliedConstraintForce = CLIB.btMultibodyLink_getAppliedConstraintForce,
	MultibodyLink_getAxisTop = CLIB.btMultibodyLink_getAxisTop,
	MultibodyLink_getCachedRotParentToThis = CLIB.btMultibodyLink_getCachedRotParentToThis,
	MultibodyLink_getCachedWorldTransform = CLIB.btMultibodyLink_getCachedWorldTransform,
	MultibodyLink_getCfgOffset = CLIB.btMultibodyLink_getCfgOffset,
	RotationalLimitMotor2_setEnableSpring = CLIB.btRotationalLimitMotor2_setEnableSpring,
	MultibodyLink_getDofOffset = CLIB.btMultibodyLink_getDofOffset,
	HingeConstraint_testLimit = CLIB.btHingeConstraint_testLimit,
	TranslationalLimitMotor_getCurrentLinearDiff = CLIB.btTranslationalLimitMotor_getCurrentLinearDiff,
	ConvexHullShape_new = CLIB.btConvexHullShape_new,
}
library.e = {
}
library.clib = CLIB
return library
