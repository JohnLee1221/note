## reference line





## scenarios

***modules/apollo_planning/src/scenarios/scenario_manager.cc***

scenario_manager用来配置和管理场景



### lane_follow

lane_follow场景配置tasks在`lane_follow_config.pb.txt`中配置，默认lane_follow场景下有14种任务

*modules/apollo_planning/conf/scenario/lane_follow_config.pb.txt* 

```shell
scenario_type: LANE_FOLLOW
stage_type: LANE_FOLLOW_DEFAULT_STAGE
stage_config: {
  stage_type: LANE_FOLLOW_DEFAULT_STAGE
  enabled: true
  task_type: LANE_CHANGE_DECIDER
  task_type: PATH_REUSE_DECIDER
  task_type: PATH_LANE_BORROW_DECIDER
  task_type: PATH_BOUNDS_DECIDER
  task_type: PIECEWISE_JERK_PATH_OPTIMIZER
  task_type: PATH_ASSESSMENT_DECIDER
  task_type: PATH_DECIDER
  task_type: RULE_BASED_STOP_DECIDER
  task_type: ST_BOUNDS_DECIDER
  task_type: SPEED_BOUNDS_PRIORI_DECIDER
  task_type: SPEED_HEURISTIC_OPTIMIZER
  task_type: SPEED_DECIDER
  task_type: SPEED_BOUNDS_FINAL_DECIDER
  # task_type: PIECEWISE_JERK_SPEED_OPTIMIZER
  task_type: PIECEWISE_JERK_NONLINEAR_SPEED_OPTIMIZER
  # task_type: RSS_DECIDER

  task_config: {
    task_type: LANE_CHANGE_DECIDER
    lane_change_decider_config {
      enable_lane_change_urgency_check: true
    }
  }
  task_config: {
    task_type: PATH_REUSE_DECIDER
    path_reuse_decider_config {
      reuse_path: false
    }
  }
  task_config: {
    task_type: PATH_LANE_BORROW_DECIDER
    path_lane_borrow_decider_config {
      allow_lane_borrowing: true
    }
  }
  task_config: {
    task_type: PATH_BOUNDS_DECIDER
  }
  task_config: {
    task_type: PIECEWISE_JERK_PATH_OPTIMIZER
  }
  task_config: {
    task_type: PATH_ASSESSMENT_DECIDER
  }
  task_config: {
    task_type: PATH_DECIDER
  }
  task_config: {
    task_type: ST_BOUNDS_DECIDER
  }
  task_config: {
    task_type: SPEED_BOUNDS_PRIORI_DECIDER
  }
  task_config: {
    task_type: SPEED_BOUNDS_FINAL_DECIDER
  }
  task_config: {
    task_type: SPEED_HEURISTIC_OPTIMIZER
  }
  task_config: {
    task_type: SPEED_DECIDER
  }
  task_config: {
    task_type: PIECEWISE_JERK_SPEED_OPTIMIZER
  }
  task_config: {
    # task_type: RSS_DECIDER
  }
  task_config: {
    task_type: RULE_BASED_STOP_DECIDER
  }
  task_config: {
    task_type: PIECEWISE_JERK_NONLINEAR_SPEED_OPTIMIZER
  }
}

```
