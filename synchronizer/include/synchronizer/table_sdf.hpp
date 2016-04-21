#ifndef TABLE_SDF_HPP
#define TABLE_SDF_HPP

#include <ros/ros.h>
#include <arc_utilities/eigen_helpers_conversions.hpp>
#include <arc_utilities/voxel_grid.hpp>
#include <sdf_tools/collision_map.hpp>
#include <sdf_tools/sdf.hpp>
#include <sdf_tools/sdf_builder.hpp>
#include <visualization_msgs/Marker.h>

namespace controls_project
{
    class TableSDF
    {
        public:
            TableSDF( ros::NodeHandle& nh,
                      const std::string& table_frame = "table_frame",
                      const double table_x_size = DEFAULT_TABLE_X_SIZE,
                      const double table_y_size = DEFAULT_TABLE_Y_SIZE,
                      const double table_z_size = DEFAULT_TABLE_Z_SIZE,
                      const double table_leg_width = DEFAULT_TABLE_LEG_WIDTH,
                      const double table_thickness = DEFAULT_TABLE_THICKNESS )
                : nh_( nh )
            {
                {
                    const bool latch = true;
                    visualization_pub_ = nh_.advertise< visualization_msgs::MarkerArray >(
                                "table_marker_array", 10, latch );
                }

                ////////////////////////////////////////////////////////////////
                // Set some parameters for the SDF
                ////////////////////////////////////////////////////////////////

                // TODO: error check to see if the GRID paramters are reasonable
                // Let's center the grid around the middle of the table
                const Eigen::Translation3d origin_translation( -GRID_X_SIZE/2.0, -GRID_Y_SIZE/2.0, -GRID_Z_SIZE/2 );
                const Eigen::Quaterniond origin_rotation( 1.0, 0.0, 0.0, 0.0 );
                const Eigen::Affine3d origin_transform = origin_translation * origin_rotation;

                // "out of bounds cell" - the value for anything not explicitly defined in the grid
                const sdf_tools::COLLISION_CELL oob_cell( 0.0 );
                // Occupancy values > 0.5 are obstacles
                const sdf_tools::COLLISION_CELL obstacle_cell( 1.0 );

                ////////////////////////////////////////////////////////////////
                // Make a collision map with a table in it
                ////////////////////////////////////////////////////////////////

                sdf_tools::CollisionMapGrid collision_map(
                            origin_transform,
                            table_frame,
                            GRID_RESOLUTION,
                            GRID_X_SIZE,
                            GRID_Y_SIZE,
                            GRID_Z_SIZE,
                            oob_cell );


                // Fill in the top of the table
                for ( double x_value = -table_x_size / 2.0; x_value <= table_x_size / 2.0; x_value += GRID_RESOLUTION / 2.0 )
                {
                    for ( double y_value = -table_y_size / 2.0; y_value <= table_y_size / 2.0; y_value += GRID_RESOLUTION / 2.0 )
                    {
                        for (double z_value = table_z_size / 2.0 - table_thickness; z_value <= table_z_size / 2.0; z_value += GRID_RESOLUTION / 2.0 )
                        {
                            assert( collision_map.Set( x_value, y_value, z_value, obstacle_cell ) && "Unable to set collions value" );
                        }
                    }
                }

                // Fill in the legs of the table
                addTableLegObstacle( collision_map,
                                     -table_x_size / 2.0 + table_leg_width / 2.0,
                                     -table_y_size / 2.0 + table_leg_width / 2.0,
                                     -table_z_size / 2.0,
                                      table_leg_width,
                                      table_z_size );
                addTableLegObstacle( collision_map,
                                     -table_x_size / 2.0 + table_leg_width / 2.0,
                                      table_y_size / 2.0 - table_leg_width / 2.0,
                                     -table_z_size / 2.0,
                                      table_leg_width,
                                      table_z_size );
                addTableLegObstacle( collision_map,
                                      table_x_size / 2.0 - table_leg_width / 2.0,
                                     -table_y_size / 2.0 + table_leg_width / 2.0,
                                     -table_z_size / 2.0,
                                      table_leg_width,
                                      table_z_size );
                addTableLegObstacle( collision_map,
                                      table_x_size / 2.0 - table_leg_width / 2.0,
                                      table_y_size / 2.0 - table_leg_width / 2.0,
                                     -table_z_size / 2.0,
                                      table_leg_width,
                                      table_z_size );

                // Create a Collision map marker so that it can be published later
                std_msgs::ColorRGBA collision_color;
                collision_color.r = 0.0;
                collision_color.g = 0.0;
                collision_color.b = 1.0;
                collision_color.a = 1.0;
                std_msgs::ColorRGBA free_color;
                free_color.r = 0.0;
                free_color.g = 1.0;
                free_color.b = 0.0;
                free_color.a = 0.0;
                std_msgs::ColorRGBA unknown_color;
                unknown_color.r = 1.0;
                unknown_color.g = 1.0;
                unknown_color.b = 0.0;
                unknown_color.a = 0.0;
                visualization_msgs::Marker collision_map_marker = collision_map.ExportForDisplay(
                            collision_color, free_color, unknown_color );
                collision_map_marker.ns = "collision_map";
                collision_map_marker.id = 1;

                ////////////////////////////////////////////////////////////////
                // Convert the collision map into an SDF
                ////////////////////////////////////////////////////////////////

                // We pick a reasonable out-of-bounds value
                const float oob_value = std::numeric_limits< float >::infinity();
                // We start by extracting the SDF from the CollisionMap
                sdf_ = collision_map.ExtractSignedDistanceField( oob_value ).first;
                sdf_.Lock();

                const double alpha = 0.5;
                visualization_msgs::Marker sdf_marker = sdf_.ExportForDisplay( alpha );
                sdf_marker.ns = "sdf";
                sdf_marker.id = 1;

                visualization_msgs::MarkerArray marker_array;
                marker_array.markers = { collision_map_marker, sdf_marker };

                visualization_pub_.publish( marker_array );

            }

            inline double getDistance( const geometry_msgs::Pose& pose ) const
            {
                return (double)sdf_.Get( pose.position.x, pose.position.y, pose.position.z );
            }

            inline geometry_msgs::Vector3 getGradient( const geometry_msgs::Pose& pose ) const
            {
                std::vector< double > gradient = sdf_.GetGradient( pose.position.x, pose.position.y, pose.position.z );

                geometry_msgs::Vector3 ros_gradient;
                ros_gradient.x = 0;
                ros_gradient.y = 0;
                ros_gradient.z = 0;
                if ( gradient.size() == 3 )
                {
                    const Eigen::Vector3d eig_gradient( gradient[0], gradient[1], gradient[2] );
                    const double norm = eig_gradient.norm();
                    if ( norm != 0 )
                    {
                        ros_gradient = EigenHelpersConversions::EigenVector3dToGeometryVector3( eig_gradient / norm );
                    }
                }

                return ros_gradient;
            }

            ////////////////////////////////////////////////////////////////////
            // World parameters
            ////////////////////////////////////////////////////////////////////

            // Define the size of the table
            static constexpr double DEFAULT_TABLE_X_SIZE = 0.5; //METERS
            static constexpr double DEFAULT_TABLE_Y_SIZE = 0.5; //METERS
            static constexpr double DEFAULT_TABLE_Z_SIZE = 1.0; //METERS
            static constexpr double DEFAULT_TABLE_LEG_WIDTH = 0.05; // METERS
            static constexpr double DEFAULT_TABLE_THICKNESS = 0.05; // METERS

        private:

            ////////////////////////////////////////////////////////////////////
            // ROS interface objects
            ////////////////////////////////////////////////////////////////////

            ros::NodeHandle nh_;
            ros::Publisher visualization_pub_;

            ////////////////////////////////////////////////////////////////////
            // SDF Parameters
            ////////////////////////////////////////////////////////////////////

            // Define the size of the grid
            static constexpr double GRID_RESOLUTION = 0.02; // METERS
            static constexpr double GRID_X_SIZE = 1.5;      // METERS
            static constexpr double GRID_Y_SIZE = 1.5;      // METERS
            static constexpr double GRID_Z_SIZE = 1.5;      // METERS

            sdf_tools::SignedDistanceField sdf_;


            // Construction helper for the collision map

            void addTableLegObstacle( sdf_tools::CollisionMapGrid& collision_map,
                                      const double table_foot_x,
                                      const double table_foot_y,
                                      const double table_foot_z,
                                      const double table_leg_width,
                                      const double table_height )
            {
                const sdf_tools::COLLISION_CELL obstacle_cell(1.0);
                for ( double x_value = table_foot_x - table_leg_width / 2.0; x_value <= table_foot_x + table_leg_width / 2.0; x_value += GRID_RESOLUTION / 2.0 )
                {
                    for ( double y_value = table_foot_y - table_leg_width / 2.0; y_value <= table_foot_y + table_leg_width / 2.0; y_value += GRID_RESOLUTION / 2.0 )
                    {
                        for (double z_value = table_foot_z; z_value <= table_foot_z + table_height; z_value += GRID_RESOLUTION / 2.0 )
                        {
                            assert( collision_map.Set(x_value, y_value, z_value, obstacle_cell) && "Unable to set collions value" );
                        }
                    }
                }
            }
    };
}

#endif // TABLE_SDF_HPP
