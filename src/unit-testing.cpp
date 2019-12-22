
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