// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
//#include "example.hpp"
#include <fstream>              // File IO
#include <iostream>             // Terminal IO
#include <sstream>              // Stringstreams

// 3rd party header for writing png files
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

// Helper function for writing metadata to disk as a csv file
void metadata_to_csv(const rs2::frame& frm, const std::string& filename);

std::tuple<uint8_t, uint8_t, uint8_t> get_texcolor(rs2::video_frame texture, rs2::texture_coordinate texcoords)
{
    const int w = texture.get_width(), h = texture.get_height();
    int x = std::min(std::max(int(texcoords.u * w + .5f), 0), w - 1);
    int y = std::min(std::max(int(texcoords.v * h + .5f), 0), h - 1);
    int idx = x * texture.get_bytes_per_pixel() + y * texture.get_stride_in_bytes();
    const auto texture_data = reinterpret_cast<const uint8_t*>(texture.get_data());
    return std::tuple<uint8_t, uint8_t, uint8_t>(texture_data[idx], texture_data[idx + 1], texture_data[idx + 2]);
}
// This sample captures 30 frames and writes the last frame to disk.
// It can be useful for debugging an embedded system with no display.
int main(int argc, char* argv[]) try
{
    /*
    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Start streaming with default recommended configuration
    pipe.start();

    // Capture 30 frames to give autoexposure, etc. a chance to settle
    for (auto i = 0; i < 30; ++i) pipe.wait_for_frames();

    // Wait for the next set of frames from the camera. Now that autoexposure, etc.
    // has settled, we will write these to disk
    for (auto&& frame : pipe.wait_for_frames())
    {
        // We can only save video frames as pngs, so we skip the rest
        if (auto vf = frame.as<rs2::video_frame>())
        {
            auto stream = frame.get_profile().stream_type();
            // Use the colorizer to get an rgb image for the depth stream
            if (vf.is<rs2::depth_frame>()) vf = color_map.process(frame);

            // Write images to disk
            std::stringstream png_file;
            png_file << "rs-save-to-disk-output-" << vf.get_profile().stream_name() << ".png";
            stbi_write_png(png_file.str().c_str(), vf.get_width(), vf.get_height(),
                           vf.get_bytes_per_pixel(), vf.get_data(), vf.get_stride_in_bytes());
            std::cout << "Saved " << png_file.str() << std::endl;

            // Record per-frame metadata for UVC streams
            std::stringstream csv_file;
            csv_file << "rs-save-to-disk-output-" << vf.get_profile().stream_name()
                     << "-metadata.csv";
            metadata_to_csv(vf, csv_file.str());
        }
    }

    return EXIT_SUCCESS;
    */
    rs2::pointcloud pc;
    rs2::points points;
    rs2::pipeline pipe;

    std::cout << "Fetching frame from Camera ... \n";
    pipe.start();
    auto frames = pipe.wait_for_frames();

    auto color = frames.get_color_frame();
    if (!color)  color = frames.get_infrared_frame();

    pc.map_to(color);
    auto depth = frames.get_depth_frame();
    points = pc.calculate(depth);

    auto vertices = points.get_vertices();
    auto tex_coords = points.get_texture_coordinates();

    int h = depth.get_height();
    int w = depth.get_width();
    int r, g, b;

    std::ofstream myfile;
    std::stringstream filename;

    filename << "Frame_Data" << ".txt";
    myfile.open(filename.str());
    std::cout << "Going to write on a file ... \n";
    //while (true) {
    for (int i = 0; i < points.size(); i++)
    {
        frames = pipe.wait_for_frames();

        color = frames.get_color_frame();
        if (!color)  color = frames.get_infrared_frame();

        pc.map_to(color);
        depth = frames.get_depth_frame();
        points = pc.calculate(depth);

        vertices = points.get_vertices();
        tex_coords = points.get_texture_coordinates();

        //h = depth.get_height();
        //w = depth.get_width();

        if (vertices[i].z)
        {
            myfile << vertices[i].x << " " << vertices[i].y << " " << vertices[i].z;
            std::tuple<uint8_t, uint8_t, uint8_t> current_color;
            current_color = get_texcolor(color, tex_coords[i]);
            r = std::get<0>(current_color);
            g = std::get<1>(current_color);
            b = std::get<2>(current_color);

            myfile << " " << r << " " << g << " " << b << "\n";
        }
    }
    //}
    myfile.close();

    system("pause");
    return EXIT_SUCCESS;

}
catch (const rs2::error& e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}

void metadata_to_csv(const rs2::frame& frm, const std::string& filename)
{
    std::ofstream csv;

    csv.open(filename);

    //    std::cout << "Writing metadata to " << filename << endl;
    csv << "Stream," << rs2_stream_to_string(frm.get_profile().stream_type()) << "\nMetadata Attribute,Value\n";

    // Record all the available metadata attributes
    for (size_t i = 0; i < RS2_FRAME_METADATA_COUNT; i++)
    {
        if (frm.supports_frame_metadata((rs2_frame_metadata_value)i))
        {
            csv << rs2_frame_metadata_to_string((rs2_frame_metadata_value)i) << ","
                << frm.get_frame_metadata((rs2_frame_metadata_value)i) << "\n";
        }
    }

    csv.close();
}
