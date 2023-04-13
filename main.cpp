#include "calibrated_opencv_camera.h"
#ifdef TEK5030_REALSENSE
#include "calibrated_realsense_camera.h"
#endif
#include "lab_vo.h"
#include <iostream>


int main()
{
  try
  {
    // Choose camera.
    // constexpr int camera_id = 0;
    // auto camera = std::make_shared<CalibratedOpencvCamera>(camera_id);

    auto camera = std::make_shared<CalibratedRealSenseCamera>();

    LabVO lab(camera);
    lab.run();
  }
  catch (const std::exception& e)
  {
    std::cerr << "Caught exception:\n"
              << e.what() << "\n";
  }
  catch (...)
  {
    std::cerr << "Caught unknown exception\n";
  }

  return EXIT_SUCCESS;
}
