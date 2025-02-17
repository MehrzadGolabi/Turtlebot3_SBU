/* Include files */

#include "robotROS2FeedbackControlExample2_sfun.h"
#include "c1_robotROS2FeedbackControlExample2.h"
#include "mwmathutil.h"
#define _SF_MEX_LISTEN_FOR_CTRL_C(S)   sf_mex_listen_for_ctrl_c(S);
#ifdef utFree
#undef utFree
#endif

#ifdef utMalloc
#undef utMalloc
#endif

#ifdef __cplusplus

extern "C" void *utMalloc(size_t size);
extern "C" void utFree(void*);

#else

extern void *utMalloc(size_t size);
extern void utFree(void*);

#endif

/* Forward Declarations */

/* Type Definitions */

/* Named Constants */
const int32_T CALL_EVENT = -1;

/* Variable Declarations */

/* Variable Definitions */
static real_T _sfTime_;
static emlrtMCInfo c1_emlrtMCI = { 13, /* lineNo */
  9,                                   /* colNo */
  "sqrt",                              /* fName */
  "C:\\Program Files\\MATLAB\\R2023b\\toolbox\\eml\\lib\\matlab\\elfun\\sqrt.m"/* pName */
};

static emlrtMCInfo c1_b_emlrtMCI = { 82,/* lineNo */
  5,                                   /* colNo */
  "power",                             /* fName */
  "C:\\Program Files\\MATLAB\\R2023b\\toolbox\\eml\\lib\\matlab\\ops\\power.m"/* pName */
};

static emlrtMCInfo c1_c_emlrtMCI = { 14,/* lineNo */
  9,                                   /* colNo */
  "asin",                              /* fName */
  "C:\\Program Files\\MATLAB\\R2023b\\toolbox\\eml\\lib\\matlab\\elfun\\asin.m"/* pName */
};

static emlrtMCInfo c1_d_emlrtMCI = { 87,/* lineNo */
  33,                                  /* colNo */
  "eml_int_forloop_overflow_check",    /* fName */
  "C:\\Program Files\\MATLAB\\R2023b\\toolbox\\eml\\lib\\matlab\\eml\\eml_int_forloop_overflow_check.m"/* pName */
};

static emlrtRSInfo c1_emlrtRSI = { 50, /* lineNo */
  "quat2eul",                          /* fcnName */
  "C:\\Program Files\\MATLAB\\R2023b\\toolbox\\shared\\robotics\\robotutils\\quat2eul.m"/* pathName */
};

static emlrtRSInfo c1_b_emlrtRSI = { 75,/* lineNo */
  "quat2eul",                          /* fcnName */
  "C:\\Program Files\\MATLAB\\R2023b\\toolbox\\shared\\robotics\\robotutils\\quat2eul.m"/* pathName */
};

static emlrtRSInfo c1_c_emlrtRSI = { 76,/* lineNo */
  "quat2eul",                          /* fcnName */
  "C:\\Program Files\\MATLAB\\R2023b\\toolbox\\shared\\robotics\\robotutils\\quat2eul.m"/* pathName */
};

static emlrtRSInfo c1_d_emlrtRSI = { 77,/* lineNo */
  "quat2eul",                          /* fcnName */
  "C:\\Program Files\\MATLAB\\R2023b\\toolbox\\shared\\robotics\\robotutils\\quat2eul.m"/* pathName */
};

static emlrtRSInfo c1_e_emlrtRSI = { 15,/* lineNo */
  "normalizeRows",                     /* fcnName */
  "C:\\Program Files\\MATLAB\\R2023b\\toolbox\\shared\\robotics\\robotutilsint\\+robotics\\+internal\\normalizeRows.m"/* pathName */
};

static emlrtRSInfo c1_f_emlrtRSI = { 71,/* lineNo */
  "power",                             /* fcnName */
  "C:\\Program Files\\MATLAB\\R2023b\\toolbox\\eml\\lib\\matlab\\ops\\power.m"/* pathName */
};

static emlrtRSInfo c1_g_emlrtRSI = { 5,/* lineNo */
  "Conversion",                        /* fcnName */
  "#robotROS2FeedbackControlExample2:115"/* pathName */
};

static emlrtRTEInfo c1_emlrtRTEI = { 78,/* lineNo */
  30,                                  /* colNo */
  "quat2eul",                          /* fName */
  "C:\\Program Files\\MATLAB\\R2023b\\toolbox\\shared\\robotics\\robotutils\\quat2eul.m"/* pName */
};

static emlrtRTEInfo c1_b_emlrtRTEI = { 78,/* lineNo */
  62,                                  /* colNo */
  "quat2eul",                          /* fName */
  "C:\\Program Files\\MATLAB\\R2023b\\toolbox\\shared\\robotics\\robotutils\\quat2eul.m"/* pName */
};

static emlrtRTEInfo c1_c_emlrtRTEI = { 78,/* lineNo */
  74,                                  /* colNo */
  "quat2eul",                          /* fName */
  "C:\\Program Files\\MATLAB\\R2023b\\toolbox\\shared\\robotics\\robotutils\\quat2eul.m"/* pName */
};

static emlrtRTEInfo c1_d_emlrtRTEI = { 78,/* lineNo */
  56,                                  /* colNo */
  "quat2eul",                          /* fName */
  "C:\\Program Files\\MATLAB\\R2023b\\toolbox\\shared\\robotics\\robotutils\\quat2eul.m"/* pName */
};

static emlrtRTEInfo c1_e_emlrtRTEI = { 234,/* lineNo */
  20,                                  /* colNo */
  "ixfun",                             /* fName */
  "C:\\Program Files\\MATLAB\\R2023b\\toolbox\\eml\\eml\\+coder\\+internal\\ixfun.m"/* pName */
};

static emlrtRTEInfo c1_f_emlrtRTEI = { 236,/* lineNo */
  26,                                  /* colNo */
  "ixfun",                             /* fName */
  "C:\\Program Files\\MATLAB\\R2023b\\toolbox\\eml\\eml\\+coder\\+internal\\ixfun.m"/* pName */
};

static emlrtRTEInfo c1_g_emlrtRTEI = { 163,/* lineNo */
  14,                                  /* colNo */
  "ixfun",                             /* fName */
  "C:\\Program Files\\MATLAB\\R2023b\\toolbox\\eml\\eml\\+coder\\+internal\\ixfun.m"/* pName */
};

static emlrtRTEInfo c1_h_emlrtRTEI = { 236,/* lineNo */
  5,                                   /* colNo */
  "ixfun",                             /* fName */
  "C:\\Program Files\\MATLAB\\R2023b\\toolbox\\eml\\eml\\+coder\\+internal\\ixfun.m"/* pName */
};

static emlrtRTEInfo c1_i_emlrtRTEI = { 78,/* lineNo */
  24,                                  /* colNo */
  "quat2eul",                          /* fName */
  "C:\\Program Files\\MATLAB\\R2023b\\toolbox\\shared\\robotics\\robotutils\\quat2eul.m"/* pName */
};

static emlrtECInfo c1_emlrtECI = { 1,  /* nDims */
  78,                                  /* lineNo */
  24,                                  /* colNo */
  "quat2eul",                          /* fName */
  "C:\\Program Files\\MATLAB\\R2023b\\toolbox\\shared\\robotics\\robotutils\\quat2eul.m"/* pName */
};

static emlrtECInfo c1_b_emlrtECI = { -1,/* nDims */
  78,                                  /* lineNo */
  9,                                   /* colNo */
  "quat2eul",                          /* fName */
  "C:\\Program Files\\MATLAB\\R2023b\\toolbox\\shared\\robotics\\robotutils\\quat2eul.m"/* pName */
};

/* Function Declarations */
static void initialize_c1_robotROS2FeedbackControlExample2
  (SFc1_robotROS2FeedbackControlExample2InstanceStruct *chartInstance);
static void initialize_params_c1_robotROS2FeedbackControlExample2
  (SFc1_robotROS2FeedbackControlExample2InstanceStruct *chartInstance);
static void mdl_start_c1_robotROS2FeedbackControlExample2
  (SFc1_robotROS2FeedbackControlExample2InstanceStruct *chartInstance);
static void mdl_terminate_c1_robotROS2FeedbackControlExample2
  (SFc1_robotROS2FeedbackControlExample2InstanceStruct *chartInstance);
static void mdl_setup_runtime_resources_c1_robotROS2FeedbackControlExample2
  (SFc1_robotROS2FeedbackControlExample2InstanceStruct *chartInstance);
static void mdl_cleanup_runtime_resources_c1_robotROS2FeedbackControlExample2
  (SFc1_robotROS2FeedbackControlExample2InstanceStruct *chartInstance);
static void enable_c1_robotROS2FeedbackControlExample2
  (SFc1_robotROS2FeedbackControlExample2InstanceStruct *chartInstance);
static void disable_c1_robotROS2FeedbackControlExample2
  (SFc1_robotROS2FeedbackControlExample2InstanceStruct *chartInstance);
static void sf_gateway_c1_robotROS2FeedbackControlExample2
  (SFc1_robotROS2FeedbackControlExample2InstanceStruct *chartInstance);
static void ext_mode_exec_c1_robotROS2FeedbackControlExample2
  (SFc1_robotROS2FeedbackControlExample2InstanceStruct *chartInstance);
static void c1_update_jit_animation_c1_robotROS2FeedbackControlExample2
  (SFc1_robotROS2FeedbackControlExample2InstanceStruct *chartInstance);
static void c1_do_animation_call_c1_robotROS2FeedbackControlExample2
  (SFc1_robotROS2FeedbackControlExample2InstanceStruct *chartInstance);
static const mxArray *get_sim_state_c1_robotROS2FeedbackControlExample2
  (SFc1_robotROS2FeedbackControlExample2InstanceStruct *chartInstance);
static void set_sim_state_c1_robotROS2FeedbackControlExample2
  (SFc1_robotROS2FeedbackControlExample2InstanceStruct *chartInstance, const
   mxArray *c1_st);
static void initSimStructsc1_robotROS2FeedbackControlExample2
  (SFc1_robotROS2FeedbackControlExample2InstanceStruct *chartInstance);
static void initSubchartIOPointersc1_robotROS2FeedbackControlExample2
  (SFc1_robotROS2FeedbackControlExample2InstanceStruct *chartInstance);
static real_T c1_sumColumnB(SFc1_robotROS2FeedbackControlExample2InstanceStruct *
  chartInstance, real_T c1_b_x[4]);
static real_T c1_function_handle_parenReference
  (SFc1_robotROS2FeedbackControlExample2InstanceStruct *chartInstance, real_T
   c1_varargin_1, real_T c1_varargin_2);
static void c1_expand_atan2(SFc1_robotROS2FeedbackControlExample2InstanceStruct *
  chartInstance, real_T c1_a_data[], int32_T c1_a_size[1], real_T c1_b_data[],
  int32_T c1_b_size[1], real_T c1_c_data[], int32_T c1_c_size[1]);
static real_T c1_emlrt_marshallIn
  (SFc1_robotROS2FeedbackControlExample2InstanceStruct *chartInstance, const
   mxArray *c1_nullptr, const char_T *c1_identifier);
static real_T c1_b_emlrt_marshallIn
  (SFc1_robotROS2FeedbackControlExample2InstanceStruct *chartInstance, const
   mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static uint8_T c1_c_emlrt_marshallIn
  (SFc1_robotROS2FeedbackControlExample2InstanceStruct *chartInstance, const
   mxArray *c1_nullptr, const char_T *c1_identifier);
static uint8_T c1_d_emlrt_marshallIn
  (SFc1_robotROS2FeedbackControlExample2InstanceStruct *chartInstance, const
   mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void c1_slStringInitializeDynamicBuffers
  (SFc1_robotROS2FeedbackControlExample2InstanceStruct *chartInstance);
static void c1_chart_data_browse_helper
  (SFc1_robotROS2FeedbackControlExample2InstanceStruct *chartInstance, int32_T
   c1_ssIdNumber, const mxArray **c1_mxData, uint8_T *c1_isValueTooBig);
static void c1_times(SFc1_robotROS2FeedbackControlExample2InstanceStruct
                     *chartInstance, real_T c1_in1_data[], int32_T c1_in1_size[1],
                     real_T c1_in2_data[], int32_T c1_in2_size[1]);
static void init_dsm_address_info
  (SFc1_robotROS2FeedbackControlExample2InstanceStruct *chartInstance);
static void init_simulink_io_address
  (SFc1_robotROS2FeedbackControlExample2InstanceStruct *chartInstance);

/* Function Definitions */
static void initialize_c1_robotROS2FeedbackControlExample2
  (SFc1_robotROS2FeedbackControlExample2InstanceStruct *chartInstance)
{
  sf_is_first_init_cond(chartInstance->S);
  sim_mode_is_external(chartInstance->S);
  chartInstance->c1_doneDoubleBufferReInit = false;
  chartInstance->c1_sfEvent = CALL_EVENT;
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void initialize_params_c1_robotROS2FeedbackControlExample2
  (SFc1_robotROS2FeedbackControlExample2InstanceStruct *chartInstance)
{
}

static void mdl_start_c1_robotROS2FeedbackControlExample2
  (SFc1_robotROS2FeedbackControlExample2InstanceStruct *chartInstance)
{
  sim_mode_is_external(chartInstance->S);
}

static void mdl_terminate_c1_robotROS2FeedbackControlExample2
  (SFc1_robotROS2FeedbackControlExample2InstanceStruct *chartInstance)
{
}

static void mdl_setup_runtime_resources_c1_robotROS2FeedbackControlExample2
  (SFc1_robotROS2FeedbackControlExample2InstanceStruct *chartInstance)
{
  static const uint32_T c1_decisionTxtEndIdx = 0U;
  static const uint32_T c1_decisionTxtStartIdx = 0U;
  sfSetAnimationVectors(chartInstance->S, chartInstance->c1_JITStateAnimation,
                        chartInstance->c1_JITTransitionAnimation);
  setDataBrowseFcn(chartInstance->S, (void *)&c1_chart_data_browse_helper);
  chartInstance->c1_RuntimeVar = sfListenerCacheSimStruct(chartInstance->S);
  sfListenerInitializeRuntimeVars(chartInstance->c1_RuntimeVar,
    &chartInstance->c1_IsDebuggerActive,
    &chartInstance->c1_IsSequenceViewerPresent, 0, 0,
    &chartInstance->c1_mlFcnLineNumber, &chartInstance->c1_IsHeatMapPresent, 0);
  covrtCreateStateflowInstanceData(chartInstance->c1_covrtInstance, 1U, 0U, 1U,
    12U);
  covrtChartInitFcn(chartInstance->c1_covrtInstance, 0U, false, false, false);
  covrtStateInitFcn(chartInstance->c1_covrtInstance, 0U, 0U, false, false, false,
                    0U, &c1_decisionTxtStartIdx, &c1_decisionTxtEndIdx);
  covrtTransInitFcn(chartInstance->c1_covrtInstance, 0U, 0, NULL, NULL, 0U, NULL);
  covrtEmlInitFcn(chartInstance->c1_covrtInstance, "", 4U, 0U, 1U, 0U, 0U, 0U,
                  0U, 0U, 0U, 0U, 0U, 0U);
  covrtEmlFcnInitFcn(chartInstance->c1_covrtInstance, 4U, 0U, 0U,
                     "eML_blk_kernel", 0, -1, 148);
}

static void mdl_cleanup_runtime_resources_c1_robotROS2FeedbackControlExample2
  (SFc1_robotROS2FeedbackControlExample2InstanceStruct *chartInstance)
{
  sfListenerLightTerminate(chartInstance->c1_RuntimeVar);
  covrtDeleteStateflowInstanceData(chartInstance->c1_covrtInstance);
}

static void enable_c1_robotROS2FeedbackControlExample2
  (SFc1_robotROS2FeedbackControlExample2InstanceStruct *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void disable_c1_robotROS2FeedbackControlExample2
  (SFc1_robotROS2FeedbackControlExample2InstanceStruct *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void sf_gateway_c1_robotROS2FeedbackControlExample2
  (SFc1_robotROS2FeedbackControlExample2InstanceStruct *chartInstance)
{
  static char_T c1_cv[30] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'E', 'l', 'F', 'u', 'n', 'D', 'o', 'm', 'a', 'i', 'n',
    'E', 'r', 'r', 'o', 'r' };

  static char_T c1_cv2[30] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'E', 'l', 'F', 'u', 'n', 'D', 'o', 'm', 'a', 'i', 'n',
    'E', 'r', 'r', 'o', 'r' };

  static char_T c1_cv1[4] = { 's', 'q', 'r', 't' };

  static char_T c1_cv3[4] = { 'a', 's', 'i', 'n' };

  emlrtStack c1_b_st;
  emlrtStack c1_c_st;
  emlrtStack c1_d_st;
  emlrtStack c1_st = { NULL,           /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  const mxArray *c1_f_y = NULL;
  const mxArray *c1_g_y = NULL;
  const mxArray *c1_h_y = NULL;
  const mxArray *c1_r_y = NULL;
  const mxArray *c1_s_y = NULL;
  const mxArray *c1_t_y = NULL;
  real_T c1_d_y[4];
  real_T c1_q[4];
  real_T c1_eul[3];
  real_T c1_b_tmp_data[1];
  real_T c1_b_x_data[1];
  real_T c1_dv[1];
  real_T c1_dv1[1];
  real_T c1_x_data[1];
  real_T c1_y_data[1];
  real_T c1_a;
  real_T c1_aSinInput;
  real_T c1_b;
  real_T c1_b_a;
  real_T c1_b_b;
  real_T c1_b_w;
  real_T c1_b_x;
  real_T c1_b_y;
  real_T c1_b_yaw;
  real_T c1_b_z;
  real_T c1_c_a;
  real_T c1_c_b;
  real_T c1_c_x;
  real_T c1_c_y;
  real_T c1_d_a;
  real_T c1_d_b;
  real_T c1_d_x;
  real_T c1_e_a;
  real_T c1_e_b;
  real_T c1_e_x;
  real_T c1_e_y;
  real_T c1_f_a;
  real_T c1_f_x;
  real_T c1_g_a;
  real_T c1_g_x;
  real_T c1_h_a;
  real_T c1_h_x;
  real_T c1_i_x;
  real_T c1_i_y;
  real_T c1_j_a;
  real_T c1_j_x;
  real_T c1_j_y;
  real_T c1_k_a;
  real_T c1_k_x;
  real_T c1_k_y;
  real_T c1_m_a;
  real_T c1_m_x;
  real_T c1_m_y;
  real_T c1_o_a;
  real_T c1_o_y;
  real_T c1_p_a;
  real_T c1_p_y;
  real_T c1_q_a;
  real_T c1_qw;
  real_T c1_qx;
  real_T c1_qy;
  real_T c1_qz;
  real_T c1_r;
  real_T c1_r_a;
  real_T c1_s_a;
  real_T c1_t_a;
  real_T c1_u_a;
  int32_T c1_c_tmp_size[2];
  int32_T c1_b_tmp_size[1];
  int32_T c1_b_x_size[1];
  int32_T c1_c_tmp_data[1];
  int32_T c1_c_trueCount[1];
  int32_T c1_e_trueCount[1];
  int32_T c1_iv[1];
  int32_T c1_iv1[1];
  int32_T c1_iv2[1];
  int32_T c1_tmp_data[1];
  int32_T c1_tmp_size[1];
  int32_T c1_x_size[1];
  int32_T c1_y_size[1];
  int32_T c1_b_end;
  int32_T c1_b_i;
  int32_T c1_b_k;
  int32_T c1_b_loop_ub;
  int32_T c1_b_nx;
  int32_T c1_b_partialTrueCount;
  int32_T c1_b_trueCount;
  int32_T c1_b_varargin_2;
  int32_T c1_c_end;
  int32_T c1_c_i;
  int32_T c1_c_k;
  int32_T c1_c_loop_ub;
  int32_T c1_d_end;
  int32_T c1_d_i;
  int32_T c1_d_k;
  int32_T c1_d_loop_ub;
  int32_T c1_d_trueCount;
  int32_T c1_e_end;
  int32_T c1_e_i;
  int32_T c1_e_loop_ub;
  int32_T c1_end;
  int32_T c1_f_end;
  int32_T c1_f_i;
  int32_T c1_f_loop_ub;
  int32_T c1_f_trueCount;
  int32_T c1_g_end;
  int32_T c1_g_i;
  int32_T c1_g_loop_ub;
  int32_T c1_g_trueCount;
  int32_T c1_h_i;
  int32_T c1_h_loop_ub;
  int32_T c1_i;
  int32_T c1_i1;
  int32_T c1_i2;
  int32_T c1_i3;
  int32_T c1_i4;
  int32_T c1_i5;
  int32_T c1_i6;
  int32_T c1_i7;
  int32_T c1_i8;
  int32_T c1_i9;
  int32_T c1_ia;
  int32_T c1_ic;
  int32_T c1_j_i;
  int32_T c1_k;
  int32_T c1_k_i;
  int32_T c1_loop_ub;
  int32_T c1_m_i;
  int32_T c1_partialTrueCount;
  int32_T c1_trueCount;
  int32_T c1_varargin_1;
  int32_T c1_varargin_2;
  int32_T c1_varargin_4;
  boolean_T c1_b_b1;
  boolean_T c1_b_p;
  boolean_T c1_c_p;
  boolean_T c1_d_p;
  boolean_T c1_e_p;
  boolean_T c1_f_b;
  boolean_T c1_mask1;
  boolean_T c1_mask2;
  boolean_T c1_p;
  boolean_T c1_samesize;
  c1_st.tls = chartInstance->c1_fEmlrtCtx;
  c1_b_st.prev = &c1_st;
  c1_b_st.tls = c1_st.tls;
  c1_c_st.prev = &c1_b_st;
  c1_c_st.tls = c1_b_st.tls;
  c1_d_st.prev = &c1_c_st;
  c1_d_st.tls = c1_c_st.tls;
  covrtSigUpdateFcn(chartInstance->c1_covrtInstance, 3U, *chartInstance->c1_w);
  covrtSigUpdateFcn(chartInstance->c1_covrtInstance, 2U, *chartInstance->c1_z);
  covrtSigUpdateFcn(chartInstance->c1_covrtInstance, 1U, *chartInstance->c1_y);
  covrtSigUpdateFcn(chartInstance->c1_covrtInstance, 0U, *chartInstance->c1_x);
  _sfTime_ = sf_get_time(chartInstance->S);
  chartInstance->c1_JITTransitionAnimation[0] = 0U;
  chartInstance->c1_sfEvent = CALL_EVENT;
  c1_b_x = *chartInstance->c1_x;
  c1_b_y = *chartInstance->c1_y;
  c1_b_z = *chartInstance->c1_z;
  c1_b_w = *chartInstance->c1_w;
  covrtEmlFcnEval(chartInstance->c1_covrtInstance, 4U, 0, 0);
  c1_b_st.site = &c1_g_emlrtRSI;
  c1_q[0] = c1_b_w;
  c1_q[1] = c1_b_x;
  c1_q[2] = c1_b_y;
  c1_q[3] = c1_b_z;
  c1_c_st.site = &c1_emlrtRSI;
  for (c1_k = 0; c1_k < 4; c1_k++) {
    c1_b_k = c1_k;
    c1_a = c1_q[c1_b_k];
    c1_e_y = c1_a * c1_a;
    c1_d_y[c1_b_k] = c1_e_y;
  }

  c1_c_y = c1_sumColumnB(chartInstance, c1_d_y);
  c1_d_st.site = &c1_e_emlrtRSI;
  c1_c_x = c1_c_y;
  c1_d_x = c1_c_x;
  c1_e_x = c1_d_x;
  if (c1_e_x < 0.0) {
    c1_p = true;
  } else {
    c1_p = false;
  }

  c1_b_p = c1_p;
  if (c1_b_p) {
    c1_f_y = NULL;
    sf_mex_assign(&c1_f_y, sf_mex_create("y", c1_cv, 10, 0U, 1U, 0U, 2, 1, 30),
                  false);
    c1_g_y = NULL;
    sf_mex_assign(&c1_g_y, sf_mex_create("y", c1_cv, 10, 0U, 1U, 0U, 2, 1, 30),
                  false);
    c1_h_y = NULL;
    sf_mex_assign(&c1_h_y, sf_mex_create("y", c1_cv1, 10, 0U, 1U, 0U, 2, 1, 4),
                  false);
    sf_mex_call(&c1_d_st, &c1_emlrtMCI, "error", 0U, 2U, 14, c1_f_y, 14,
                sf_mex_call(&c1_d_st, NULL, "getString", 1U, 1U, 14, sf_mex_call
      (&c1_d_st, NULL, "message", 1U, 2U, 14, c1_g_y, 14, c1_h_y)));
  }

  c1_d_x = muDoubleScalarSqrt(c1_d_x);
  c1_b = 1.0 / c1_d_x;
  for (c1_i = 0; c1_i < 4; c1_i++) {
    c1_d_y[c1_i] = c1_q[c1_i];
  }

  c1_b_b = c1_b;
  for (c1_c_k = 0; c1_c_k < 4; c1_c_k++) {
    c1_ia = c1_c_k;
    c1_ic = c1_c_k;
    c1_c_b = c1_b_b;
    c1_varargin_1 = c1_ic + 1;
    c1_varargin_2 = c1_ia + 1;
    c1_d_b = c1_c_b;
    c1_b_varargin_2 = c1_varargin_1 - 1;
    c1_varargin_4 = c1_varargin_2 - 1;
    c1_q[c1_b_varargin_2] = c1_d_y[c1_varargin_4] * c1_d_b;
  }

  c1_qw = c1_q[0];
  c1_qx = c1_q[1];
  c1_qy = c1_q[2];
  c1_qz = c1_q[3];
  c1_aSinInput = -2.0 * (c1_qx * c1_qz - c1_qw * c1_qy);
  c1_mask1 = (c1_aSinInput >= 0.99999999999999778);
  c1_mask2 = (c1_aSinInput <= -0.99999999999999778);
  c1_dv[0] = c1_aSinInput;
  c1_end = 1;
  for (c1_b_i = 0; c1_b_i < c1_end; c1_b_i++) {
    if (c1_aSinInput >= 0.99999999999999778) {
      c1_dv[c1_b_i] = 1.0;
    }
  }

  c1_aSinInput = c1_dv[0];
  c1_dv1[0] = c1_aSinInput;
  c1_b_end = 1;
  for (c1_c_i = 0; c1_c_i < c1_b_end; c1_c_i++) {
    if (c1_mask2) {
      c1_dv1[c1_c_i] = -1.0;
    }
  }

  c1_aSinInput = c1_dv1[0];
  c1_c_st.site = &c1_b_emlrtRSI;
  c1_b_a = c1_qw;
  c1_d_st.site = &c1_f_emlrtRSI;
  c1_c_a = c1_b_a;
  c1_d_a = c1_c_a;
  c1_e_a = c1_d_a;
  c1_i_y = c1_e_a * c1_e_a;
  c1_c_st.site = &c1_b_emlrtRSI;
  c1_f_a = c1_qx;
  c1_d_st.site = &c1_f_emlrtRSI;
  c1_g_a = c1_f_a;
  c1_h_a = c1_g_a;
  c1_j_a = c1_h_a;
  c1_j_y = c1_j_a * c1_j_a;
  c1_c_st.site = &c1_b_emlrtRSI;
  c1_k_a = c1_qy;
  c1_d_st.site = &c1_f_emlrtRSI;
  c1_m_a = c1_k_a;
  c1_o_a = c1_m_a;
  c1_p_a = c1_o_a;
  c1_k_y = c1_p_a * c1_p_a;
  c1_c_st.site = &c1_b_emlrtRSI;
  c1_q_a = c1_qz;
  c1_d_st.site = &c1_f_emlrtRSI;
  c1_r_a = c1_q_a;
  c1_s_a = c1_r_a;
  c1_t_a = c1_s_a;
  c1_m_y = c1_t_a * c1_t_a;
  c1_c_st.site = &c1_d_emlrtRSI;
  c1_d_st.site = &c1_f_emlrtRSI;
  c1_c_st.site = &c1_d_emlrtRSI;
  c1_d_st.site = &c1_f_emlrtRSI;
  c1_c_st.site = &c1_d_emlrtRSI;
  c1_d_st.site = &c1_f_emlrtRSI;
  c1_c_st.site = &c1_d_emlrtRSI;
  c1_d_st.site = &c1_f_emlrtRSI;
  c1_o_y = 2.0 * (c1_qx * c1_qy + c1_qw * c1_qz);
  c1_f_x = ((c1_i_y + c1_j_y) - c1_k_y) - c1_m_y;
  c1_u_a = c1_o_y;
  c1_e_b = c1_f_x;
  c1_p_y = c1_u_a;
  c1_g_x = c1_e_b;
  c1_r = muDoubleScalarAtan2(c1_p_y, c1_g_x);
  c1_c_st.site = &c1_c_emlrtRSI;
  c1_h_x = c1_aSinInput;
  c1_i_x = c1_h_x;
  c1_j_x = c1_i_x;
  c1_f_b = (c1_j_x < -1.0);
  c1_b_b1 = (c1_j_x > 1.0);
  if (c1_f_b || c1_b_b1) {
    c1_c_p = true;
  } else {
    c1_c_p = false;
  }

  c1_d_p = c1_c_p;
  if (c1_d_p) {
    c1_r_y = NULL;
    sf_mex_assign(&c1_r_y, sf_mex_create("y", c1_cv2, 10, 0U, 1U, 0U, 2, 1, 30),
                  false);
    c1_s_y = NULL;
    sf_mex_assign(&c1_s_y, sf_mex_create("y", c1_cv2, 10, 0U, 1U, 0U, 2, 1, 30),
                  false);
    c1_t_y = NULL;
    sf_mex_assign(&c1_t_y, sf_mex_create("y", c1_cv3, 10, 0U, 1U, 0U, 2, 1, 4),
                  false);
    sf_mex_call(&c1_c_st, &c1_c_emlrtMCI, "error", 0U, 2U, 14, c1_r_y, 14,
                sf_mex_call(&c1_c_st, NULL, "getString", 1U, 1U, 14, sf_mex_call
      (&c1_c_st, NULL, "message", 1U, 2U, 14, c1_s_y, 14, c1_t_y)));
  }

  c1_eul[0] = c1_r;
  c1_c_end = 1;
  c1_trueCount = 0;
  for (c1_d_i = 0; c1_d_i < c1_c_end; c1_d_i++) {
    if (c1_mask1 || c1_mask2) {
      c1_trueCount++;
    }
  }

  c1_tmp_size[0] = c1_trueCount;
  c1_partialTrueCount = 0;
  for (c1_e_i = 0; c1_e_i < c1_c_end; c1_e_i++) {
    if (c1_mask1 || c1_mask2) {
      c1_tmp_data[c1_partialTrueCount] = c1_e_i;
      c1_partialTrueCount++;
    }
  }

  c1_x_size[0] = c1_tmp_size[0];
  c1_loop_ub = c1_tmp_size[0] - 1;
  for (c1_i1 = 0; c1_i1 <= c1_loop_ub; c1_i1++) {
    c1_x_data[c1_i1] = c1_aSinInput;
  }

  c1_b_nx = c1_x_size[0];
  c1_i2 = static_cast<uint8_T>(c1_b_nx) - 1;
  for (c1_d_k = 0; c1_d_k <= c1_i2; c1_d_k++) {
    c1_k_x = c1_x_data[0];
    c1_m_x = c1_k_x;
    c1_m_x = muDoubleScalarSign(c1_m_x);
    c1_x_data[0] = c1_m_x;
  }

  c1_b_loop_ub = c1_x_size[0] - 1;
  for (c1_i3 = 0; c1_i3 <= c1_b_loop_ub; c1_i3++) {
    c1_x_data[c1_i3] = -c1_x_data[c1_i3] * 2.0;
  }

  c1_d_end = 1;
  c1_b_trueCount = 0;
  for (c1_f_i = 0; c1_f_i < c1_d_end; c1_f_i++) {
    if (c1_mask1 || c1_mask2) {
      c1_b_trueCount++;
    }
  }

  c1_c_trueCount[0] = c1_b_trueCount;
  c1_y_size[0] = c1_c_trueCount[0];
  c1_c_loop_ub = c1_c_trueCount[0] - 1;
  for (c1_i4 = 0; c1_i4 <= c1_c_loop_ub; c1_i4++) {
    c1_y_data[c1_i4] = c1_qx;
  }

  c1_e_end = 1;
  c1_d_trueCount = 0;
  for (c1_g_i = 0; c1_g_i < c1_e_end; c1_g_i++) {
    if (c1_mask1 || c1_mask2) {
      c1_d_trueCount++;
    }
  }

  c1_e_trueCount[0] = c1_d_trueCount;
  c1_b_x_size[0] = c1_e_trueCount[0];
  c1_d_loop_ub = c1_e_trueCount[0] - 1;
  for (c1_i5 = 0; c1_i5 <= c1_d_loop_ub; c1_i5++) {
    c1_b_x_data[c1_i5] = c1_qw;
  }

  c1_e_p = (static_cast<real_T>(c1_y_size[0]) == static_cast<real_T>
            (c1_b_x_size[0]));
  c1_samesize = c1_e_p;
  if (c1_samesize) {
    c1_b_tmp_size[0] = c1_y_size[0];
    c1_e_loop_ub = c1_y_size[0] - 1;
    for (c1_i6 = 0; c1_i6 <= c1_e_loop_ub; c1_i6++) {
      c1_b_tmp_data[c1_i6] = c1_function_handle_parenReference(chartInstance,
        c1_y_data[c1_i6], c1_b_x_data[c1_i6]);
    }
  } else {
    c1_expand_atan2(chartInstance, c1_y_data, c1_y_size, c1_b_x_data,
                    c1_b_x_size, c1_b_tmp_data, c1_b_tmp_size);
  }

  if ((c1_x_size[0] != c1_b_tmp_size[0]) && ((c1_x_size[0] != 1) &&
       (c1_b_tmp_size[0] != 1))) {
    emlrtDimSizeImpxCheckR2021b(c1_x_size[0], c1_b_tmp_size[0], &c1_emlrtECI,
      &c1_b_st);
  }

  c1_f_end = 1;
  c1_f_trueCount = 0;
  for (c1_h_i = 0; c1_h_i < c1_f_end; c1_h_i++) {
    if (c1_mask1 || c1_mask2) {
      c1_f_trueCount++;
    }
  }

  c1_tmp_size[0] = c1_f_trueCount;
  c1_b_partialTrueCount = 0;
  for (c1_j_i = 0; c1_j_i < c1_f_end; c1_j_i++) {
    if (c1_mask1 || c1_mask2) {
      c1_tmp_data[c1_b_partialTrueCount] = c1_j_i;
      c1_b_partialTrueCount++;
    }
  }

  if (c1_x_size[0] == c1_b_tmp_size[0]) {
    c1_f_loop_ub = c1_x_size[0] - 1;
    for (c1_i7 = 0; c1_i7 <= c1_f_loop_ub; c1_i7++) {
      c1_x_data[c1_i7] *= c1_b_tmp_data[c1_i7];
    }
  } else {
    c1_times(chartInstance, c1_x_data, c1_x_size, c1_b_tmp_data, c1_b_tmp_size);
  }

  c1_iv[0] = c1_tmp_size[0];
  emlrtSubAssignSizeCheckR2012b(&c1_iv[0], 1, &c1_x_size[0], 1, &c1_b_emlrtECI,
    &c1_b_st);
  c1_iv1[0] = c1_tmp_size[0];
  c1_g_loop_ub = c1_iv1[0] - 1;
  for (c1_i8 = 0; c1_i8 <= c1_g_loop_ub; c1_i8++) {
    c1_eul[c1_tmp_data[c1_i8]] = c1_x_data[0];
  }

  c1_g_end = 1;
  c1_g_trueCount = 0;
  for (c1_k_i = 0; c1_k_i < c1_g_end; c1_k_i++) {
    if (c1_mask1 || c1_mask2) {
      c1_g_trueCount++;
    }
  }

  c1_c_tmp_size[1] = c1_g_trueCount;
  for (c1_m_i = 0; c1_m_i < c1_g_end; c1_m_i++) {
    if (c1_mask1 || c1_mask2) {
      c1_c_tmp_data[0] = c1_m_i;
    }
  }

  c1_iv2[0] = c1_c_tmp_size[1];
  c1_h_loop_ub = c1_iv2[0] - 1;
  for (c1_i9 = 0; c1_i9 <= c1_h_loop_ub; c1_i9++) {
    c1_eul[2 + c1_c_tmp_data[c1_i9]] = 0.0;
  }

  c1_b_yaw = c1_eul[0];
  *chartInstance->c1_yaw = c1_b_yaw;
  c1_do_animation_call_c1_robotROS2FeedbackControlExample2(chartInstance);
  covrtSigUpdateFcn(chartInstance->c1_covrtInstance, 4U, *chartInstance->c1_yaw);
}

static void ext_mode_exec_c1_robotROS2FeedbackControlExample2
  (SFc1_robotROS2FeedbackControlExample2InstanceStruct *chartInstance)
{
}

static void c1_update_jit_animation_c1_robotROS2FeedbackControlExample2
  (SFc1_robotROS2FeedbackControlExample2InstanceStruct *chartInstance)
{
}

static void c1_do_animation_call_c1_robotROS2FeedbackControlExample2
  (SFc1_robotROS2FeedbackControlExample2InstanceStruct *chartInstance)
{
  sfDoAnimationWrapper(chartInstance->S, false, true);
  sfDoAnimationWrapper(chartInstance->S, false, false);
}

static const mxArray *get_sim_state_c1_robotROS2FeedbackControlExample2
  (SFc1_robotROS2FeedbackControlExample2InstanceStruct *chartInstance)
{
  const mxArray *c1_b_y = NULL;
  const mxArray *c1_c_y = NULL;
  const mxArray *c1_d_y = NULL;
  const mxArray *c1_st;
  c1_st = NULL;
  c1_st = NULL;
  c1_b_y = NULL;
  sf_mex_assign(&c1_b_y, sf_mex_createcellmatrix(2, 1), false);
  c1_c_y = NULL;
  sf_mex_assign(&c1_c_y, sf_mex_create("y", chartInstance->c1_yaw, 0, 0U, 0U, 0U,
    0), false);
  sf_mex_setcell(c1_b_y, 0, c1_c_y);
  c1_d_y = NULL;
  sf_mex_assign(&c1_d_y, sf_mex_create("y",
    &chartInstance->c1_is_active_c1_robotROS2FeedbackControlExample2, 3, 0U, 0U,
    0U, 0), false);
  sf_mex_setcell(c1_b_y, 1, c1_d_y);
  sf_mex_assign(&c1_st, c1_b_y, false);
  return c1_st;
}

static void set_sim_state_c1_robotROS2FeedbackControlExample2
  (SFc1_robotROS2FeedbackControlExample2InstanceStruct *chartInstance, const
   mxArray *c1_st)
{
  const mxArray *c1_u;
  chartInstance->c1_doneDoubleBufferReInit = true;
  c1_u = sf_mex_dup(c1_st);
  *chartInstance->c1_yaw = c1_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c1_u, 0)), "yaw");
  chartInstance->c1_is_active_c1_robotROS2FeedbackControlExample2 =
    c1_c_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c1_u, 1)),
    "is_active_c1_robotROS2FeedbackControlExample2");
  sf_mex_destroy(&c1_u);
  sf_mex_destroy(&c1_st);
}

static void initSimStructsc1_robotROS2FeedbackControlExample2
  (SFc1_robotROS2FeedbackControlExample2InstanceStruct *chartInstance)
{
}

static void initSubchartIOPointersc1_robotROS2FeedbackControlExample2
  (SFc1_robotROS2FeedbackControlExample2InstanceStruct *chartInstance)
{
}

static real_T c1_sumColumnB(SFc1_robotROS2FeedbackControlExample2InstanceStruct *
  chartInstance, real_T c1_b_x[4])
{
  real_T c1_b_y;
  int32_T c1_b_k;
  int32_T c1_k;
  c1_b_y = c1_b_x[0];
  for (c1_k = 0; c1_k < 3; c1_k++) {
    c1_b_k = c1_k;
    c1_b_y += c1_b_x[c1_b_k + 1];
  }

  return c1_b_y;
}

static real_T c1_function_handle_parenReference
  (SFc1_robotROS2FeedbackControlExample2InstanceStruct *chartInstance, real_T
   c1_varargin_1, real_T c1_varargin_2)
{
  real_T c1_b_x;
  real_T c1_b_y;
  real_T c1_c_x;
  real_T c1_c_y;
  c1_b_x = c1_varargin_1;
  c1_b_y = c1_varargin_2;
  c1_c_y = c1_b_x;
  c1_c_x = c1_b_y;
  return muDoubleScalarAtan2(c1_c_y, c1_c_x);
}

static void c1_expand_atan2(SFc1_robotROS2FeedbackControlExample2InstanceStruct *
  chartInstance, real_T c1_a_data[], int32_T c1_a_size[1], real_T c1_b_data[],
  int32_T c1_b_size[1], real_T c1_c_data[], int32_T c1_c_size[1])
{
  real_T c1_b_c_data[1];
  real_T c1_b_x;
  real_T c1_b_y;
  real_T c1_r;
  int32_T c1_csz[2];
  int32_T c1_i;
  int32_T c1_k;
  int32_T c1_loop_ub;
  int32_T c1_sak;
  int32_T c1_sbk;
  int32_T c1_sck;
  c1_sak = c1_a_size[0];
  c1_sbk = c1_b_size[0];
  if (c1_sbk == 1) {
    c1_sck = c1_sak;
  } else {
    c1_sck = 0;
  }

  c1_csz[0] = c1_sck;
  c1_c_size[0] = c1_csz[0];
  if (c1_c_size[0] != 0) {
    c1_loop_ub = c1_c_size[0] - 1;
    for (c1_i = 0; c1_i <= c1_loop_ub; c1_i++) {
      c1_b_c_data[c1_i] = c1_c_data[c1_i];
    }

    for (c1_k = 0; c1_k < 1; c1_k++) {
      c1_b_y = c1_a_data[0];
      c1_b_x = c1_b_data[0];
      c1_r = muDoubleScalarAtan2(c1_b_y, c1_b_x);
      c1_b_c_data[0] = c1_r;
    }

    c1_c_size[0] = 1;
    c1_c_data[0] = c1_b_c_data[0];
  }
}

const mxArray
  *sf_c1_robotROS2FeedbackControlExample2_get_eml_resolved_functions_info()
{
  const mxArray *c1_nameCaptureInfo = NULL;
  c1_nameCaptureInfo = NULL;
  sf_mex_assign(&c1_nameCaptureInfo, sf_mex_create("nameCaptureInfo", NULL, 0,
    0U, 1U, 0U, 2, 0, 1), false);
  return c1_nameCaptureInfo;
}

static real_T c1_emlrt_marshallIn
  (SFc1_robotROS2FeedbackControlExample2InstanceStruct *chartInstance, const
   mxArray *c1_nullptr, const char_T *c1_identifier)
{
  emlrtMsgIdentifier c1_thisId;
  real_T c1_b_y;
  c1_thisId.fIdentifier = const_cast<const char_T *>(c1_identifier);
  c1_thisId.fParent = NULL;
  c1_thisId.bParentIsCell = false;
  c1_b_y = c1_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_nullptr),
    &c1_thisId);
  sf_mex_destroy(&c1_nullptr);
  return c1_b_y;
}

static real_T c1_b_emlrt_marshallIn
  (SFc1_robotROS2FeedbackControlExample2InstanceStruct *chartInstance, const
   mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  real_T c1_b_y;
  real_T c1_d;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_d, 1, 0, 0U, 0, 0U, 0);
  c1_b_y = c1_d;
  sf_mex_destroy(&c1_u);
  return c1_b_y;
}

static uint8_T c1_c_emlrt_marshallIn
  (SFc1_robotROS2FeedbackControlExample2InstanceStruct *chartInstance, const
   mxArray *c1_nullptr, const char_T *c1_identifier)
{
  emlrtMsgIdentifier c1_thisId;
  uint8_T c1_b_y;
  c1_thisId.fIdentifier = const_cast<const char_T *>(c1_identifier);
  c1_thisId.fParent = NULL;
  c1_thisId.bParentIsCell = false;
  c1_b_y = c1_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_nullptr),
    &c1_thisId);
  sf_mex_destroy(&c1_nullptr);
  return c1_b_y;
}

static uint8_T c1_d_emlrt_marshallIn
  (SFc1_robotROS2FeedbackControlExample2InstanceStruct *chartInstance, const
   mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  uint8_T c1_b_u;
  uint8_T c1_b_y;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_b_u, 1, 3, 0U, 0, 0U, 0);
  c1_b_y = c1_b_u;
  sf_mex_destroy(&c1_u);
  return c1_b_y;
}

static void c1_slStringInitializeDynamicBuffers
  (SFc1_robotROS2FeedbackControlExample2InstanceStruct *chartInstance)
{
}

static void c1_chart_data_browse_helper
  (SFc1_robotROS2FeedbackControlExample2InstanceStruct *chartInstance, int32_T
   c1_ssIdNumber, const mxArray **c1_mxData, uint8_T *c1_isValueTooBig)
{
  real_T c1_b_d1;
  real_T c1_d;
  real_T c1_d2;
  real_T c1_d3;
  real_T c1_d4;
  *c1_mxData = NULL;
  *c1_mxData = NULL;
  *c1_isValueTooBig = 0U;
  switch (c1_ssIdNumber) {
   case 8U:
    c1_d = *chartInstance->c1_yaw;
    sf_mex_assign(c1_mxData, sf_mex_create("mxData", &c1_d, 0, 0U, 0U, 0U, 0),
                  false);
    break;

   case 4U:
    c1_b_d1 = *chartInstance->c1_x;
    sf_mex_assign(c1_mxData, sf_mex_create("mxData", &c1_b_d1, 0, 0U, 0U, 0U, 0),
                  false);
    break;

   case 5U:
    c1_d2 = *chartInstance->c1_y;
    sf_mex_assign(c1_mxData, sf_mex_create("mxData", &c1_d2, 0, 0U, 0U, 0U, 0),
                  false);
    break;

   case 6U:
    c1_d3 = *chartInstance->c1_z;
    sf_mex_assign(c1_mxData, sf_mex_create("mxData", &c1_d3, 0, 0U, 0U, 0U, 0),
                  false);
    break;

   case 7U:
    c1_d4 = *chartInstance->c1_w;
    sf_mex_assign(c1_mxData, sf_mex_create("mxData", &c1_d4, 0, 0U, 0U, 0U, 0),
                  false);
    break;
  }
}

static void c1_times(SFc1_robotROS2FeedbackControlExample2InstanceStruct
                     *chartInstance, real_T c1_in1_data[], int32_T c1_in1_size[1],
                     real_T c1_in2_data[], int32_T c1_in2_size[1])
{
  real_T c1_b_in1_data[1];
  int32_T c1_b_in1_size[1];
  int32_T c1_aux_0_0;
  int32_T c1_aux_1_0;
  int32_T c1_b_loop_ub;
  int32_T c1_i;
  int32_T c1_i1;
  int32_T c1_i2;
  int32_T c1_loop_ub;
  int32_T c1_stride_0_0;
  int32_T c1_stride_1_0;
  if (c1_in2_size[0] == 1) {
    c1_b_in1_size[0] = c1_in1_size[0];
  } else {
    c1_b_in1_size[0] = c1_in2_size[0];
  }

  c1_stride_0_0 = static_cast<int32_T>(c1_in1_size[0] != 1);
  c1_stride_1_0 = static_cast<int32_T>(c1_in2_size[0] != 1);
  c1_aux_0_0 = 0;
  c1_aux_1_0 = 0;
  if (c1_in2_size[0] == 1) {
    c1_i = c1_in1_size[0];
  } else {
    c1_i = c1_in2_size[0];
  }

  c1_loop_ub = c1_i - 1;
  for (c1_i1 = 0; c1_i1 <= c1_loop_ub; c1_i1++) {
    c1_b_in1_data[c1_i1] = c1_in1_data[c1_aux_0_0] * c1_in2_data[c1_aux_1_0];
    c1_aux_1_0 += c1_stride_1_0;
    c1_aux_0_0 += c1_stride_0_0;
  }

  c1_in1_size[0] = c1_b_in1_size[0];
  c1_b_loop_ub = c1_b_in1_size[0] - 1;
  for (c1_i2 = 0; c1_i2 <= c1_b_loop_ub; c1_i2++) {
    c1_in1_data[c1_i2] = c1_b_in1_data[c1_i2];
  }
}

static void init_dsm_address_info
  (SFc1_robotROS2FeedbackControlExample2InstanceStruct *chartInstance)
{
}

static void init_simulink_io_address
  (SFc1_robotROS2FeedbackControlExample2InstanceStruct *chartInstance)
{
  chartInstance->c1_covrtInstance = (CovrtStateflowInstance *)
    sfrtGetCovrtInstance(chartInstance->S);
  chartInstance->c1_fEmlrtCtx = (void *)sfrtGetEmlrtCtx(chartInstance->S);
  chartInstance->c1_yaw = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 1);
  chartInstance->c1_x = (real_T *)ssGetInputPortSignal_wrapper(chartInstance->S,
    0);
  chartInstance->c1_y = (real_T *)ssGetInputPortSignal_wrapper(chartInstance->S,
    1);
  chartInstance->c1_z = (real_T *)ssGetInputPortSignal_wrapper(chartInstance->S,
    2);
  chartInstance->c1_w = (real_T *)ssGetInputPortSignal_wrapper(chartInstance->S,
    3);
}

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* SFunction Glue Code */
void sf_c1_robotROS2FeedbackControlExample2_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(3929911495U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(523633026U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(245678528U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(2787871479U);
}

mxArray *sf_c1_robotROS2FeedbackControlExample2_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,1);
  mxSetCell(mxcell3p, 0, mxCreateString(
             "coder.internal.BoundedEmxArrayExternalDependency"));
  return(mxcell3p);
}

mxArray *sf_c1_robotROS2FeedbackControlExample2_jit_fallback_info(void)
{
  const char *infoFields[] = { "fallbackType", "fallbackReason",
    "hiddenFallbackType", "hiddenFallbackReason", "incompatibleSymbol" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 5, infoFields);
  mxArray *fallbackType = mxCreateString("late");
  mxArray *fallbackReason = mxCreateString("ir_vars");
  mxArray *hiddenFallbackType = mxCreateString("");
  mxArray *hiddenFallbackReason = mxCreateString("");
  mxArray *incompatibleSymbol = mxCreateString("");
  mxSetField(mxInfo, 0, infoFields[0], fallbackType);
  mxSetField(mxInfo, 0, infoFields[1], fallbackReason);
  mxSetField(mxInfo, 0, infoFields[2], hiddenFallbackType);
  mxSetField(mxInfo, 0, infoFields[3], hiddenFallbackReason);
  mxSetField(mxInfo, 0, infoFields[4], incompatibleSymbol);
  return mxInfo;
}

mxArray *sf_c1_robotROS2FeedbackControlExample2_updateBuildInfo_args_info(void)
{
  mxArray *mxBIArgs = mxCreateCellMatrix(1,0);
  return mxBIArgs;
}

static const mxArray *sf_get_sim_state_info_c1_robotROS2FeedbackControlExample2
  (void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  mxArray *mxVarInfo = sf_mex_decode(
    "eNpjYPT0ZQACPiDmYGJgYAPRQMzEAAGsUD4jVIwRLs4CF1cA4pLKglSQeHFRsmcKkM5LzAXzE0s"
    "rPPPS8sHmWzAgzGfDYj4jkvmcUHEI+GBPmX4FB5B+AyT9LFj0MyPpFwDyKhPLwe6GhQ+l9pOvH2"
    "J/AgH366K4H8LPLI5PTC7JLEuNTzaML8pPyi8J8g82cktNTUlKTM52zs8rKcrPca1IzC3ISTWC2"
    "gMCAH3/ISQ="
    );
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c1_robotROS2FeedbackControlExample2_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static const char* sf_get_instance_specialization(void)
{
  return "sLVYACgLlpG6lxJe23oQPgH";
}

static void sf_opaque_initialize_c1_robotROS2FeedbackControlExample2(void
  *chartInstanceVar)
{
  initialize_params_c1_robotROS2FeedbackControlExample2
    ((SFc1_robotROS2FeedbackControlExample2InstanceStruct*) chartInstanceVar);
  initialize_c1_robotROS2FeedbackControlExample2
    ((SFc1_robotROS2FeedbackControlExample2InstanceStruct*) chartInstanceVar);
}

static void sf_opaque_enable_c1_robotROS2FeedbackControlExample2(void
  *chartInstanceVar)
{
  enable_c1_robotROS2FeedbackControlExample2
    ((SFc1_robotROS2FeedbackControlExample2InstanceStruct*) chartInstanceVar);
}

static void sf_opaque_disable_c1_robotROS2FeedbackControlExample2(void
  *chartInstanceVar)
{
  disable_c1_robotROS2FeedbackControlExample2
    ((SFc1_robotROS2FeedbackControlExample2InstanceStruct*) chartInstanceVar);
}

static void sf_opaque_gateway_c1_robotROS2FeedbackControlExample2(void
  *chartInstanceVar)
{
  sf_gateway_c1_robotROS2FeedbackControlExample2
    ((SFc1_robotROS2FeedbackControlExample2InstanceStruct*) chartInstanceVar);
}

static const mxArray*
  sf_opaque_get_sim_state_c1_robotROS2FeedbackControlExample2(SimStruct* S)
{
  return get_sim_state_c1_robotROS2FeedbackControlExample2
    ((SFc1_robotROS2FeedbackControlExample2InstanceStruct *)
     sf_get_chart_instance_ptr(S));    /* raw sim ctx */
}

static void sf_opaque_set_sim_state_c1_robotROS2FeedbackControlExample2
  (SimStruct* S, const mxArray *st)
{
  set_sim_state_c1_robotROS2FeedbackControlExample2
    ((SFc1_robotROS2FeedbackControlExample2InstanceStruct*)
     sf_get_chart_instance_ptr(S), st);
}

static void
  sf_opaque_cleanup_runtime_resources_c1_robotROS2FeedbackControlExample2(void
  *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc1_robotROS2FeedbackControlExample2InstanceStruct*)
                    chartInstanceVar)->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_robotROS2FeedbackControlExample2_optimization_info();
    }

    mdl_cleanup_runtime_resources_c1_robotROS2FeedbackControlExample2
      ((SFc1_robotROS2FeedbackControlExample2InstanceStruct*) chartInstanceVar);
    ((SFc1_robotROS2FeedbackControlExample2InstanceStruct*) chartInstanceVar)->
      ~SFc1_robotROS2FeedbackControlExample2InstanceStruct();
    utFree(chartInstanceVar);
    if (ssGetUserData(S)!= NULL) {
      sf_free_ChartRunTimeInfo(S);
    }

    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_mdl_start_c1_robotROS2FeedbackControlExample2(void
  *chartInstanceVar)
{
  mdl_start_c1_robotROS2FeedbackControlExample2
    ((SFc1_robotROS2FeedbackControlExample2InstanceStruct*) chartInstanceVar);
  if (chartInstanceVar) {
    sf_reset_warnings_ChartRunTimeInfo
      (((SFc1_robotROS2FeedbackControlExample2InstanceStruct*)chartInstanceVar
       )->S);
  }
}

static void sf_opaque_mdl_terminate_c1_robotROS2FeedbackControlExample2(void
  *chartInstanceVar)
{
  mdl_terminate_c1_robotROS2FeedbackControlExample2
    ((SFc1_robotROS2FeedbackControlExample2InstanceStruct*) chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c1_robotROS2FeedbackControlExample2(SimStruct
  *S)
{
  mdlProcessParamsCommon(S);
  if (sf_machine_global_initializer_called()) {
    initialize_params_c1_robotROS2FeedbackControlExample2
      ((SFc1_robotROS2FeedbackControlExample2InstanceStruct*)
       sf_get_chart_instance_ptr(S));
    initSubchartIOPointersc1_robotROS2FeedbackControlExample2
      ((SFc1_robotROS2FeedbackControlExample2InstanceStruct*)
       sf_get_chart_instance_ptr(S));
  }
}

const char* sf_c1_robotROS2FeedbackControlExample2_get_post_codegen_info(void)
{
  int i;
  const char* encStrCodegen [21] = {
    "eNrdWMuOG0UULU+GUYLCJDuyQCISGxaMRCYQsUBkHD8Sg4cxac/w2EzK3dftkqurOvXwg+/gA/g",
    "FdnxClmyQ2CP2LLPkVrvteDzG7rKDSFJST09196lz7637KpNS45jg2MerdYuQPbxfxWuHTMZb+b",
    "w0d02e75LP8/mztwkJZQQxiMB2u2xE/IawSYsqmmjiPwRN4DFoya1hUjREVxbHMtEFBSLEBVKpj",
    "BevZonlTPTrVoSOWX/bY2Ev6EnLowe4II1OBB//G29qTQsZq0xBaOoAkekpaeNendN4tRWUGVZ6",
    "EPa1TbxtpcEENnWq6mPLDUs51EYQNoQ2FK2g1+gbGGqgYkZ+Rnb66mCKlknKGRXFbd2jOoAUvcP",
    "AaRrh3xNr0HoFeZGvwwQ1UjHKawmv9Kgqim1xlPMY3Zp729kkpgodG8dMxM66yiYgUH/0kwK26l",
    "bkABSN4UT48YZOu9oo2+CZXxbEGpbAGVXlEP1AQ+QXg+jAOqC4tdDGZbywIGiHQ0O3FRvgHvnmj",
    "YYLpY3yhk0mnqQ3wma8tQHuqt6Qtx6KCuVc+2HbMm3CAHjGX6WGboCd8HuAtWZRW6J3uMj1jH4r",
    "2FMLObYiRcSKe+VgAZUVia8x4ReAs8SFAURo5pnos4XWxZHVRiYVDP1qs1mQ7zK2IQyoLg2hcL5",
    "WlGlAgTO/8uSNmHaBhGi0ksm0LLzCJAY3ghLdtaI6lKqPNvYtDC9s5SLBDw1RDFUwkCW5Gnr3Ge",
    "W2oMyJxqzs3ONUY5b140Wsi5+NwCENexC5Gsg4HGOexQWKbrF25bOM2g6YGVdBh4qlRSPJYkLH4",
    "ues1B6ncCr6Qg5FXckkyLuYFX4FgFmDKoGl7AGWUjWuo/DFpFbwtJ1ld9+GwdmZGk47zjcegsBq",
    "6HR11ZuGGFU1ge0mCrQNNmA/YjsiNNMGW8BxLYuBKOuDvyMv+uDdJX3wB3N98M18Ht45V7IjzeO",
    "T4NA1dB0a9nF5oySvjbLSeEimffbHc+tfL9BnF8GRSzgyw03vH87hS0t4ydx9ke/azsXvdxf4dv",
    "BJCYfDHc3h3iGrcXu5DW/8fOP3T55/VP7rp9++evan/mUdf+kSfyn73+F+3fE7x+zn8/emPdQsY",
    "w8uJTX37aM1/vHugn+4uW6efV+uxE2ePrzHR1/C4V35TSt+lK13dY28OwvyTp/fdn0bBnSWH1TY",
    "iPLzkJtTO+nT3fqfzcm7t8Ye1+b8iZC/72+Hv320uI/L7HXlgr2ukDEdXvDbbfk3x0/4n6yR/2B",
    "hvw+yPv6cumwN58Uyw37BuFvl976414XvTZfzZepXpJ7sbogr/Yf162XittXPt06+7t+vys9k4f",
    "ubr7AeZEUe9emXXjW9/iB+/cz7+fyL2dm90mM8WnJ6yF9jg99d9vYN8e/nnvab9m81Z7/8B9Mf7",
    "pYF5WM8LkyOY/njlnK/1c1eKaB6+Zns/6gn0/vRmv7l+kJ8u/mQiUgO9cGdw0/vbVOf/gGmShZF",
    ""
  };

  static char newstr [1501] = "";
  newstr[0] = '\0';
  for (i = 0; i < 21; i++) {
    strcat(newstr, encStrCodegen[i]);
  }

  return newstr;
}

static void mdlSetWorkWidths_c1_robotROS2FeedbackControlExample2(SimStruct *S)
{
  const char* newstr =
    sf_c1_robotROS2FeedbackControlExample2_get_post_codegen_info();
  sf_set_work_widths(S, newstr);
  ssSetChecksum0(S,(3574571023U));
  ssSetChecksum1(S,(1093466164U));
  ssSetChecksum2(S,(1271960805U));
  ssSetChecksum3(S,(2977162184U));
}

static void mdlRTW_c1_robotROS2FeedbackControlExample2(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlSetupRuntimeResources_c1_robotROS2FeedbackControlExample2
  (SimStruct *S)
{
  SFc1_robotROS2FeedbackControlExample2InstanceStruct *chartInstance;
  chartInstance = (SFc1_robotROS2FeedbackControlExample2InstanceStruct *)
    utMalloc(sizeof(SFc1_robotROS2FeedbackControlExample2InstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  memset(chartInstance, 0, sizeof
         (SFc1_robotROS2FeedbackControlExample2InstanceStruct));
  chartInstance = new (chartInstance)
    SFc1_robotROS2FeedbackControlExample2InstanceStruct;
  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c1_robotROS2FeedbackControlExample2;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c1_robotROS2FeedbackControlExample2;
  chartInstance->chartInfo.mdlStart =
    sf_opaque_mdl_start_c1_robotROS2FeedbackControlExample2;
  chartInstance->chartInfo.mdlTerminate =
    sf_opaque_mdl_terminate_c1_robotROS2FeedbackControlExample2;
  chartInstance->chartInfo.mdlCleanupRuntimeResources =
    sf_opaque_cleanup_runtime_resources_c1_robotROS2FeedbackControlExample2;
  chartInstance->chartInfo.enableChart =
    sf_opaque_enable_c1_robotROS2FeedbackControlExample2;
  chartInstance->chartInfo.disableChart =
    sf_opaque_disable_c1_robotROS2FeedbackControlExample2;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c1_robotROS2FeedbackControlExample2;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c1_robotROS2FeedbackControlExample2;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c1_robotROS2FeedbackControlExample2;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c1_robotROS2FeedbackControlExample2;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c1_robotROS2FeedbackControlExample2;
  chartInstance->chartInfo.extModeExec = NULL;
  chartInstance->chartInfo.restoreLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.restoreBeforeLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.storeCurrentConfiguration = NULL;
  chartInstance->chartInfo.callAtomicSubchartUserFcn = NULL;
  chartInstance->chartInfo.callAtomicSubchartAutoFcn = NULL;
  chartInstance->chartInfo.callAtomicSubchartEventFcn = NULL;
  chartInstance->S = S;
  chartInstance->chartInfo.dispatchToExportedFcn = NULL;
  sf_init_ChartRunTimeInfo(S, &(chartInstance->chartInfo), false, 0);
  init_dsm_address_info(chartInstance);
  init_simulink_io_address(chartInstance);
  if (!sim_mode_is_rtw_gen(S)) {
  }

  mdl_setup_runtime_resources_c1_robotROS2FeedbackControlExample2(chartInstance);
}

void c1_robotROS2FeedbackControlExample2_method_dispatcher(SimStruct *S, int_T
  method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_SETUP_RUNTIME_RESOURCES:
    mdlSetupRuntimeResources_c1_robotROS2FeedbackControlExample2(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c1_robotROS2FeedbackControlExample2(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c1_robotROS2FeedbackControlExample2(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c1_robotROS2FeedbackControlExample2_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
