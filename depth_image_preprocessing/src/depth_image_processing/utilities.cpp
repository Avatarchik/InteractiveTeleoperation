#include "../../include/depth_image_processing/utilities.h"

namespace Biolab
{
    Parameters::Parameters()
    {
        this->minBorderLength = 50;

        // TODO: I need to implement median filter by myself
        this->medianFilteringEnabled = true;  // This is essential (probably) for kinect 2
        this->medianFilterRadius = 3;

        // TODO: probably I need to remove radius-based outliers
        // TODO: handle salt/pepper noise

        this->buildFullCloud = true;


        this->radiusOutlierRemovalEnabled = true;
        this->radiusOutlierRemovalThreshold = 1.0f;

        this->saltRemovalEnabled = true;
        this->saltValue = 7000;
    }
}
