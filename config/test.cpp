const char* stateName[6] = {"roll", "pitch", "yaw", "x", "y", "z"};

for (int i = 0; i < 6; ++i)
{
    if (weakDirection[i])
    {
        std::cout << "V" << i
                  << " lambda=" << matE.at<float>(0, i)
                  << " vector=[";

        for (int j = 0; j < 6; ++j)
        {
            std::cout << stateName[j]
                      << ":" << matV.at<float>(i, j);

            if (j < 5)
                std::cout << ", ";
        }

        std::cout << "]" << std::endl;
    }
}