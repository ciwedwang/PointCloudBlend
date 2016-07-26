#pragma once
#include "cmm_types.h"

namespace cloud_icp_reg {
class FeatureAlign {
public:
	static double do_align_pre(const CloudPtr cloud_src , const CloudPtr cloud_dst, Eigen::Matrix4f& rot);
private:
	FeatureAlign(){}
    DISALLOW_COPY_AND_ASSIGN(FeatureAlign);
};
}