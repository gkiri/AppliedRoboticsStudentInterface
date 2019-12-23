
/* **********************Gkiri PRM space Unit Testing*************************/
void UT_sample_generation(std::vector<Polygon> obstacle_list ,img_map_def *map_param)
{
    PRM obj(obstacle_list);
    obj.generate_random_points(1.5,1.0, 100);
    std::vector<Point> free_space_points = obj.get_free_space_points();
    std::cout << " $$$$$$$$$$$$$$$$$$$$  no:of point= " << free_space_points.size() <<  std::endl;
    for (size_t i = 0; i<obstacle_list.size(); i++){
         draw_polygon(obstacle_list[i], *map_param);
    }
    for (int z=0;z<free_space_points.size();z++){
        draw_point(free_space_points[z], *map_param); 
           
    }

}

void UT_sampling_motion_plan(std::vector<Polygon> obstacle_list ,img_map_def *map_param){
    /*Gkiri  PRM sampling motion planning debugging*/
    //draw_motion_planning(obstacle_list,&map_param);
    //draw_motion_planning(inflated_obstacle_list,&map_param);

}


/*----------------------------- Dubins section--------------------------------*/
void UT_dubins_curve_test(struct arc_extract *three_seg,img_map_def *map_param)
{
    //Visualize dubins curve test
    arc_extract dubins_line;

    for(int i=0;i<3;i++){
      switch (three_seg[i].LSR)
      {
      case 0: // Left arc
        std::cout << "Left arc" << std::endl;
        //Print values
        std::cout << "Start point: " << three_seg[i].start_point.x 
                                        << ", " << three_seg[i].start_point.y << std::endl;
        std::cout << "End point: " << three_seg[i].end_point.x 
                                        << ", " << three_seg[i].end_point.y << std::endl;
        std::cout << "Radius: " << three_seg[i].radius << std::endl;
        std::cout << "Center: " << three_seg[i].center.x 
                          << ", " << three_seg[i].center.y <<std::endl;
        std::cout << "Length: " << three_seg[i].length <<std::endl;
        std::cout << "LSR: " << three_seg[i].LSR <<std::endl;

        draw_dubins_segment(three_seg[i], *map_param);
                             
        
        break;
      case 1: // Straight line
        std::cout << "Straight line" << std::endl;
        dubins_line = to_arc_extract_type(three_seg[i].start_point,
                                                three_seg[i].end_point,true);
        //Print values
        std::cout << "Start point: " << three_seg[i].start_point.x 
                                        << ", " << three_seg[i].start_point.y << std::endl;
        std::cout << "End point: " << three_seg[i].end_point.x 
                                        << ", " << three_seg[i].end_point.y << std::endl;
        std::cout << "Radius: " << three_seg[i].radius << std::endl;
        std::cout << "Center: " << three_seg[i].center.x 
                          << ", " << three_seg[i].center.y << std::endl;
        std::cout << "Length: " << three_seg[i].length << std::endl;
        std::cout << "LSR: " << three_seg[i].LSR <<std::endl;
        std::cout << "Calculated Length: " << dubins_line.length << std::endl;

        draw_dubins_segment(dubins_line,*map_param);
        break;
      case 2: // Right arc
        std::cout << "Right arc" << std::endl;
        //Print values
        std::cout << "Start point: " << three_seg[i].start_point.x 
                                        << ", " << three_seg[i].start_point.y << std::endl;
        std::cout << "End point: " << three_seg[i].end_point.x 
                                        << ", " << three_seg[i].end_point.y << std::endl;
        std::cout << "Radius: " << three_seg[i].radius << std::endl;
        std::cout << "Center: " << three_seg[i].center.x 
                          << ", " << three_seg[i].center.y << std::endl;
        std::cout << "Length: " << three_seg[i].length <<std::endl;
        std::cout << "LSR: " << three_seg[i].LSR <<std::endl;      

        draw_dubins_segment(three_seg[i], *map_param);
        break;
      
      default:
        std::cout << "Unknown LSR" << std::endl;
        break;
      }
    }

}


void arc_draw_test(img_map_def *map_param)
{
    //dubin drawing test
    arc_extract dt[2];
    //line
    dt[0].start_point = Point (0.456287, 0.599802);
    dt[0].end_point = Point (1.24375, 0.550198);
    dt[0].LSR = 1;

    //arc
    dt[1].start_point = Point (1.24375, 0.550198);
    dt[1].end_point = Point (1.25, 0.75);
    dt[1].radius = 0.1;
    dt[1].center = Point(1.34683, 0.646974);
    dt[1].LSR = 0;

    std::cout << "x pre: " <<  dt[1].center.x << std::endl;
    draw_dubins_segment(dt[0],*map_param);
    draw_dubins_segment(dt[1],*map_param);
    draw_point(dt[1].center,*map_param);
    std::cout << "x pos " << dt[1].center.x << std::endl;

    arc_param curve_angles = calculate_arc_drawing_angles(dt[1]);
    std::cout << "Curve Rotation angle: " << curve_angles.rotation_angle << std::endl;
    std::cout << "Curve Angle btw cs & ce: " << curve_angles.angle_cs_ce << std::endl;


}