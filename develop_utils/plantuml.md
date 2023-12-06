# plantuml toturial



类图关系

https://subingwen.cn/design-patterns/UML-class-diagrams/



```shell
sudo apt install libgraphviz-dev openjdk-11-jre
```







```wsd
!define MY_PAGE_SIZE 400x600
@startuml
' skinparam class {
'     BackgroundColor Blue
' }
class Geometry<class T, OGRwkbGeometryType R> {
  # data : std::shared_ptr<T>
  + getDimMinX() const : double
  + getDimMaxX() const : double
  + getDimMinY() const : double
  + getDimMaxY() const : double
  + getHeight() const : double
  + getWidth() const : double
  + Distance(const Geometry<T2, R2>& p) const : double
  + Disjoint(const Geometry<T2, R2>& geom) const : bool
  + Crosses(const Geometry<T2, R2>& geom) const : bool
  + Touches(const Geometry<T2, R2>& geom) const : bool
  + Within(const Geometry<T2, R2>& geom) const : bool
  + Intersects(const Geometry<T2, R2>& geom) const : bool
  + {static} mod_2pi(double val) : double
  + {static} getAngContinuity(double prev_val, double val) : double
  + {static} getAngContinuity(const std::vector<double>& val) : std::vector<double>
  + {static} getAngleDiffAbs(double a, double b) : double
  + isEmpty() const : bool
  + exportToWkt() const : string
  + importFromWkt(const std::string& text) : void
  + exportToGML() const : string
  + exportToKML() const : string
  + exportToJson() const : string
}
class Geometries<class SAMETYPE, class T, OGRwkbGeometryType R, class CHILDRENTYPE> {
  + getArea() const : double
  + clone() const : SAMETYPE
  class Iterator {}
  class ConstIterator {}
  + begin() : Iterator
  + end() : Iterator
  + begin() const : ConstIterator
  + end() const : ConstIterator
  + begin(const SAMETYPE *proSelf) : ConstIterator
  + end(const SAMETYPE *poSelf) : ConstIterator
}
Geometry <|-- Geometries
class Cell
class Cells
class LinearRing
class LineString
class MultiLineString
class MultiPoint
Geometries <|-- Cell
Geometries <|-- Cells
Geometries <|-- LinearRing
Geometries <|-- LineString
Geometries <|-- MultiLineString
Geometries <|-- MultiPoint
class Point
Geometry <|-- Point
struct Field
struct OptimizationParams
struct Path #Green
struct Robot
struct Route
struct Strip
struct Swath
struct Swaths #Green
Point <-- Path
Point <-- Robot
Swaths <-- Route
MultiPoint <-- Route
LineString <-- Swath
Swath <-- Swaths
Cell <-- Strip
Point <-- Field
Cells <-- Field
' }

package utils {
  class Parser
  class Random
  class Transform
  class Visualizer
}

namespace obj {
class BaseObjective<typename T> {
  + computeCostWithMinimizingSign<typename T1> (
      const T1& t1) : double
  + computeCostWithMinimizingSign<typename T1, typename T2> (
      const T1& t1, const T2& t2) : double
  + computeCostWithMinimizingSign<typename T1, typename T2, typename T3> (
      const T1& t1, const T2& t2, const T3& t3) : double
  + computeCostWithMinimizingSign<typename T1, typename T2, typename T3, typename T4> (
      const T1& t1, const T2& t2, const T3& t3, const T4& t4) : double
}
class RPObjective {
  + {abstract} computeCost(const F2CPoint& p1, const F2CPoint& p2) : double
  + {abstract} computeCost(const F2CPoint& p1, double ang1, const F2CPoint& p2) : double
  + {abstract} computeCost(const F2CPoint& p1, double ang1, const F2CPoint& p2, double ang2) : double
  + {abstract} computeCost(const F2CPoint& p1, const F2CPoint& p2, double ang2) : double
  + {abstract} computeCost(const F2CSwath& s, const F2CPoint& p) : double
  + {abstract} computeCost(const F2CSwath& s1, const F2CSwath& s2) : double
  + {abstract} computeCost(const F2CSwath& s, const F2CPoint& p, double ang) : double
  + {abstract} computeCost(const F2CPoint& p, const F2CSwath& s) : double
  + {abstract} computeCost(const F2CPoint& p, double ang, const F2CSwath& s) : double
  + {abstract} computeCost(const std::vector<F2CPoint>& ps) : double
  + {abstract} computeCost(const F2CMultiPoint& ps) : double
  + {abstract} computeCost(const F2CSwath& s, const F2CMultiPoint& ps) : double
  + {abstract} computeCost(const F2CSwaths& s, const F2CMultiPoint& ps) : double
  + {abstract} computeCost(const F2CMultiPoint& ps, const F2CSwath& s) : double
  + {abstract} computeCost(const F2CMultiPoint& ps, const F2CSwaths& s) : double
  + {abstract} computeCost(const F2CSwath& s) : double
  + {abstract} computeCost(const F2CSwaths& swaths) : double
  + {abstract} computeCost(const F2CRoute& r) : double
}
class PPObjective {
  + {abstract} computeCost(const F2CPath& path) : double
}
class HGObjective {
  + {abstract} computeCost(const F2CCell& total_cell, const F2CCell& rem_cell) : double
  + {abstract} computeCost(const F2CCells& total_cell, const F2CCell& rem_cell) : double
  + {abstract} computeCost(const F2CCell& total_cell, const F2CCells& rem_cell) : double
  + {abstract} computeCost(const F2CCells& total_cell, const F2CCells& rem_cell) : double
}
class SGObjective {
  + {abstract} computeCost(const F2CSwath&) : double
  + {abstract} computeCost(const F2CSwaths&) : double
  + {abstract} computeCost(const F2CSwathsByCells& swaths) : double
  + {abstract} computeCost(const F2CCell& c, const F2CSwath& s) : double
  + {abstract} computeCost(const F2CCell& c, const F2CSwaths& s) : double
  + {abstract} computeCost(const F2CCell& c, const F2CSwathsByCells& swaths) : double
  + {abstract} computeCost(const F2CCells& c, const F2CSwath& s) : double
  + {abstract} computeCost(const F2CCells& c, const F2CSwaths& s) : double
  + {abstract} computeCost(const F2CCells& c, const F2CSwathsByCells& swaths) : double
}
BaseObjective <|-- RPObjective
BaseObjective <|-- PPObjective
BaseObjective <|-- HGObjective
BaseObjective <|-- SGObjective
class CompleteTurnPathObj<class T, class R = PPObjective> {
  + turn_planner : TurningBase
  + robot : Robot
  + computeCost(const F2CPoint& p1, const F2CPoint& p2) : double
  + setRobot(const F2CRobot& params) : void
  + setTurnPlanner(const T& turner) : void
}
PPObjective <-- CompleteTurnPathObj
class DirectDistPathObj {
  + computeCost(const F2CPoint& p1, double ang1, const F2CPoint& p2, double ang2) : double
}
RPObjective <|-- CompleteTurnPathObj
RPObjective <|-- DirectDistPathObj
class FieldCoverage {
  + computeCost(const F2CCell& poly, const F2CSwaths& swaths) : double
  + computeCost(const F2CCells& poly, const F2CSwaths& swaths) : double
  + isMinimizing() const : bool
}
class NSwath {
  + computeCost(const F2CSwath& s) : double
  + computeCost(const F2CSwaths& swaths) : double
}
class Overlaps {
  + computeCost(const F2CCell& poly, const F2CSwaths& swaths) : double
  + computeCost(const F2CCells& poly, const F2CSwaths& swaths) : double
}
class SwathLength {
  + computeCost(const F2CSwath& s) : double
}
SGObjective <|-- FieldCoverage
SGObjective <|-- NSwath
SGObjective <|-- Overlaps
SGObjective <|-- SwathLength
class PathLength
PPObjective <|-- PathLength
class RemArea #Yellow { 
  + isMinimizing() : bool
}
HGObjective <|-- RemArea
}

namespace hg {
class HeadlandGeneratorBase {
  + {abstract} generateHeadlands(const F2CCells& field, double dist_headland) : F2CCells = 0
}
class ConstHL {
  + generateHeadlands(const F2CCells& field, double dist_headland) : F2CCells
}
HeadlandGeneratorBase <|-- ConstHL
}

Cells <... hg.HeadlandGeneratorBase

namespace pp {
class TurningBase {
  + using_cache : bool
  # path_cache_ : std::map<std::vector<int>, F2CPath>
  + createTurn(const F2CRobot& robot, const F2CPoint& start_pos, double start_angle,
        const F2CPoint& end_pos, double end_angle, double max_headland_width = 1e5) : F2CPath
  + createTurnIfNotCached(const F2CRobot& robot, double dist_start_pos,
        double start_angle, double end_angle, double max_headland_width = 1e5) :  F2CPath
  + {abstract} createSimpleTurn(const F2CRobot& robot, double dist_start_pos,
        double start_angle, double end_angle, double max_headland_width = 1e5) : F2CPath = 0
  + {static} transformToNormalTurn(const F2CPoint& start_pos, double start_angle,
        const F2CPoint& end_pos, double end_angle) : std::vector<double>
  + {static} bool isTurnValid(const F2CPath& path, double dist_start_end, double end_angle,
        double max_dist_error = 0.05, double max_rot_error = 0.1) : bool
}
class DubinsCurvesCC {
  + discretization : double
  + createSimpleTurn(const F2CRobot& robot, double dist_start_pos,
        double start_angle, double end_angle, double max_headland_width = 1e5) : F2CPath  
}
class DubinsCurves {
  + discretization : double
  + createSimpleTurn(const F2CRobot& robot, double dist_start_pos,
        double start_angle, double end_angle, double max_headland_width = 1e5) : F2CPath  
}
class ReedsSheppCurvesHC {
  + discretization : double
  + createSimpleTurn(const F2CRobot& robot, double dist_start_pos,
        double start_angle, double end_angle, double max_headland_width = 1e5) : F2CPath  
}
class ReedsSheppCurves {
  + discretization : double
  + createSimpleTurn(const F2CRobot& robot, double dist_start_pos,
        double start_angle, double end_angle, double max_headland_width = 1e5) : F2CPath  
}
TurningBase <|-- DubinsCurvesCC
TurningBase <|-- DubinsCurves
TurningBase <|-- ReedsSheppCurvesHC
TurningBase <|-- ReedsSheppCurves
class PathPlanning {
  + turn_point_dist : double
  + searchBestPath(const F2CRobot& robot, const F2CSwaths& swaths,
        TurningBase& turn) : F2CPath
}
' TurningBase <.. PathPlanning
PathPlanning ..> TurningBase
}
Path <.. pp.PathPlanning
pp.TurningBase <-- obj.CompleteTurnPathObj

namespace rp {
class SingleCellSwathsOrderBase {
  + {abstract} genSortedSwaths(const F2CSwaths& swaths,
        uint32_t variant = 0) const : F2CSwaths
  # {abstract} changeStartPoint(F2CSwaths& swaths,
        uint32_t variant) const : void
  # {abstract} sortSwaths(F2CSwaths& swaths) const : void = 0
}

class BoustrophedonOrder {
  # sortSwaths(F2CSwaths& swaths) const : void
}
class SnakeOrder {
  # sortSwaths(F2CSwaths& swaths) const : void
}
class SpiralOrder {
  # sortSwaths(F2CSwaths& swaths) const : void
}
class CustomOrder {
  # sortSwaths(F2CSwaths& swaths) const : void
}
SingleCellSwathsOrderBase <|-- BoustrophedonOrder
SingleCellSwathsOrderBase <|-- SnakeOrder
SingleCellSwathsOrderBase <|-- SpiralOrder
SingleCellSwathsOrderBase <|-- CustomOrder
}
Swaths <.. rp.SingleCellSwathsOrderBase

namespace sg {
class SwathGeneratorBase<typename T> {
  + {abstract} generateBestSwaths(f2c::obj::SGObjective& obj,
      double op_width, const F2CCell& poly) : F2CSwaths = 0
  + {abstract} generateBestSwaths(f2c::obj::SGObjective& obj,
      double op_width, const F2CCells& polys) : F2CSwathsByCells
  + {abstract} generateSwaths(double angle, double op_width,
      const F2CCell& poly) : F2CSwaths
  + {abstract} generateSwaths(double angle, double op_width,
      const F2CCells& polys) : F2CSwathsByCells
}
class BruteForce {
  + step_angle : double
  + generateBestSwaths(f2c::obj::SGObjective& obj,
      double op_width, const F2CCell& poly) : F2CSwaths
}
SwathGeneratorBase <|-- BruteForce
}
obj.SGObjective  <.. sg.SwathGeneratorBase
Swaths <.. sg.SwathGeneratorBase


@endumlxxxxxxxxxx @startumlclass PlanningNode {  - Planning planning_  + void Run()  + void Reset()  - void RunOnce()  - ADC!define MY_PAGE_SIZE 400x600@startuml' skinparam class {'     BackgroundColor Blue' }class Geometry<class T, OGRwkbGeometryType R> {  # data : std::shared_ptr<T>  + getDimMinX() const : double  + getDimMaxX() const : double  + getDimMinY() const : double  + getDimMaxY() const : double  + getHeight() const : double  + getWidth() const : double  + Distance(const Geometry<T2, R2>& p) const : double  + Disjoint(const Geometry<T2, R2>& geom) const : bool  + Crosses(const Geometry<T2, R2>& geom) const : bool  + Touches(const Geometry<T2, R2>& geom) const : bool  + Within(const Geometry<T2, R2>& geom) const : bool  + Intersects(const Geometry<T2, R2>& geom) const : bool  + {static} mod_2pi(double val) : double  + {static} getAngContinuity(double prev_val, double val) : double  + {static} getAngContinuity(const std::vector<double>& val) : std::vector<double>  + {static} getAngleDiffAbs(double a, double b) : double  + isEmpty() const : bool  + exportToWkt() const : string  + importFromWkt(const std::string& text) : void  + exportToGML() const : string  + exportToKML() const : string  + exportToJson() const : string}class Geometries<class SAMETYPE, class T, OGRwkbGeometryType R, class CHILDRENTYPE> {  + getArea() const : double  + clone() const : SAMETYPE  class Iterator {}  class ConstIterator {}  + begin() : Iterator  + end() : Iterator  + begin() const : ConstIterator  + end() const : ConstIterator  + begin(const SAMETYPE *proSelf) : ConstIterator  + end(const SAMETYPE *poSelf) : ConstIterator}Geometry <|-- Geometriesclass Cellclass Cellsclass LinearRingclass LineStringclass MultiLineStringclass MultiPointGeometries <|-- CellGeometries <|-- CellsGeometries <|-- LinearRingGeometries <|-- LineStringGeometries <|-- MultiLineStringGeometries <|-- MultiPointclass PointGeometry <|-- Pointstruct Fieldstruct OptimizationParamsstruct Path #Greenstruct Robotstruct Routestruct Stripstruct Swathstruct Swaths #GreenPoint <-- PathPoint <-- RobotSwaths <-- RouteMultiPoint <-- RouteLineString <-- SwathSwath <-- SwathsCell <-- StripPoint <-- FieldCells <-- Field' }package utils {  class Parser  class Random  class Transform  class Visualizer}namespace obj {class BaseObjective<typename T> {  + computeCostWithMinimizingSign<typename T1> (      const T1& t1) : double  + computeCostWithMinimizingSign<typename T1, typename T2> (      const T1& t1, const T2& t2) : double  + computeCostWithMinimizingSign<typename T1, typename T2, typename T3> (      const T1& t1, const T2& t2, const T3& t3) : double  + computeCostWithMinimizingSign<typename T1, typename T2, typename T3, typename T4> (      const T1& t1, const T2& t2, const T3& t3, const T4& t4) : double}class RPObjective {  + {abstract} computeCost(const F2CPoint& p1, const F2CPoint& p2) : double  + {abstract} computeCost(const F2CPoint& p1, double ang1, const F2CPoint& p2) : double  + {abstract} computeCost(const F2CPoint& p1, double ang1, const F2CPoint& p2, double ang2) : double  + {abstract} computeCost(const F2CPoint& p1, const F2CPoint& p2, double ang2) : double  + {abstract} computeCost(const F2CSwath& s, const F2CPoint& p) : double  + {abstract} computeCost(const F2CSwath& s1, const F2CSwath& s2) : double  + {abstract} computeCost(const F2CSwath& s, const F2CPoint& p, double ang) : double  + {abstract} computeCost(const F2CPoint& p, const F2CSwath& s) : double  + {abstract} computeCost(const F2CPoint& p, double ang, const F2CSwath& s) : double  + {abstract} computeCost(const std::vector<F2CPoint>& ps) : double  + {abstract} computeCost(const F2CMultiPoint& ps) : double  + {abstract} computeCost(const F2CSwath& s, const F2CMultiPoint& ps) : double  + {abstract} computeCost(const F2CSwaths& s, const F2CMultiPoint& ps) : double  + {abstract} computeCost(const F2CMultiPoint& ps, const F2CSwath& s) : double  + {abstract} computeCost(const F2CMultiPoint& ps, const F2CSwaths& s) : double  + {abstract} computeCost(const F2CSwath& s) : double  + {abstract} computeCost(const F2CSwaths& swaths) : double  + {abstract} computeCost(const F2CRoute& r) : double}class PPObjective {  + {abstract} computeCost(const F2CPath& path) : double}class HGObjective {  + {abstract} computeCost(const F2CCell& total_cell, const F2CCell& rem_cell) : double  + {abstract} computeCost(const F2CCells& total_cell, const F2CCell& rem_cell) : double  + {abstract} computeCost(const F2CCell& total_cell, const F2CCells& rem_cell) : double  + {abstract} computeCost(const F2CCells& total_cell, const F2CCells& rem_cell) : double}class SGObjective {  + {abstract} computeCost(const F2CSwath&) : double  + {abstract} computeCost(const F2CSwaths&) : double  + {abstract} computeCost(const F2CSwathsByCells& swaths) : double  + {abstract} computeCost(const F2CCell& c, const F2CSwath& s) : double  + {abstract} computeCost(const F2CCell& c, const F2CSwaths& s) : double  + {abstract} computeCost(const F2CCell& c, const F2CSwathsByCells& swaths) : double  + {abstract} computeCost(const F2CCells& c, const F2CSwath& s) : double  + {abstract} computeCost(const F2CCells& c, const F2CSwaths& s) : double  + {abstract} computeCost(const F2CCells& c, const F2CSwathsByCells& swaths) : double}BaseObjective <|-- RPObjectiveBaseObjective <|-- PPObjectiveBaseObjective <|-- HGObjectiveBaseObjective <|-- SGObjectiveclass CompleteTurnPathObj<class T, class R = PPObjective> {  + turn_planner : TurningBase  + robot : Robot  + computeCost(const F2CPoint& p1, const F2CPoint& p2) : double  + setRobot(const F2CRobot& params) : void  + setTurnPlanner(const T& turner) : void}PPObjective <-- CompleteTurnPathObjclass DirectDistPathObj {  + computeCost(const F2CPoint& p1, double ang1, const F2CPoint& p2, double ang2) : double}RPObjective <|-- CompleteTurnPathObjRPObjective <|-- DirectDistPathObjclass FieldCoverage {  + computeCost(const F2CCell& poly, const F2CSwaths& swaths) : double  + computeCost(const F2CCells& poly, const F2CSwaths& swaths) : double  + isMinimizing() const : bool}class NSwath {  + computeCost(const F2CSwath& s) : double  + computeCost(const F2CSwaths& swaths) : double}class Overlaps {  + computeCost(const F2CCell& poly, const F2CSwaths& swaths) : double  + computeCost(const F2CCells& poly, const F2CSwaths& swaths) : double}class SwathLength {  + computeCost(const F2CSwath& s) : double}SGObjective <|-- FieldCoverageSGObjective <|-- NSwathSGObjective <|-- OverlapsSGObjective <|-- SwathLengthclass PathLengthPPObjective <|-- PathLengthclass RemArea #Yellow {   + isMinimizing() : bool}HGObjective <|-- RemArea}namespace hg {class HeadlandGeneratorBase {  + {abstract} generateHeadlands(const F2CCells& field, double dist_headland) : F2CCells = 0}class ConstHL {  + generateHeadlands(const F2CCells& field, double dist_headland) : F2CCells}HeadlandGeneratorBase <|-- ConstHL}Cells <... hg.HeadlandGeneratorBasenamespace pp {class TurningBase {  + using_cache : bool  # path_cache_ : std::map<std::vector<int>, F2CPath>  + createTurn(const F2CRobot& robot, const F2CPoint& start_pos, double start_angle,        const F2CPoint& end_pos, double end_angle, double max_headland_width = 1e5) : F2CPath  + createTurnIfNotCached(const F2CRobot& robot, double dist_start_pos,        double start_angle, double end_angle, double max_headland_width = 1e5) :  F2CPath  + {abstract} createSimpleTurn(const F2CRobot& robot, double dist_start_pos,        double start_angle, double end_angle, double max_headland_width = 1e5) : F2CPath = 0  + {static} transformToNormalTurn(const F2CPoint& start_pos, double start_angle,        const F2CPoint& end_pos, double end_angle) : std::vector<double>  + {static} bool isTurnValid(const F2CPath& path, double dist_start_end, double end_angle,        double max_dist_error = 0.05, double max_rot_error = 0.1) : bool}class DubinsCurvesCC {  + discretization : double  + createSimpleTurn(const F2CRobot& robot, double dist_start_pos,        double start_angle, double end_angle, double max_headland_width = 1e5) : F2CPath  }class DubinsCurves {  + discretization : double  + createSimpleTurn(const F2CRobot& robot, double dist_start_pos,        double start_angle, double end_angle, double max_headland_width = 1e5) : F2CPath  }class ReedsSheppCurvesHC {  + discretization : double  + createSimpleTurn(const F2CRobot& robot, double dist_start_pos,        double start_angle, double end_angle, double max_headland_width = 1e5) : F2CPath  }class ReedsSheppCurves {  + discretization : double  + createSimpleTurn(const F2CRobot& robot, double dist_start_pos,        double start_angle, double end_angle, double max_headland_width = 1e5) : F2CPath  }TurningBase <|-- DubinsCurvesCCTurningBase <|-- DubinsCurvesTurningBase <|-- ReedsSheppCurvesHCTurningBase <|-- ReedsSheppCurvesclass PathPlanning {  + turn_point_dist : double  + searchBestPath(const F2CRobot& robot, const F2CSwaths& swaths,        TurningBase& turn) : F2CPath}' TurningBase <.. PathPlanningPathPlanning ..> TurningBase}Path <.. pp.PathPlanningpp.TurningBase <-- obj.CompleteTurnPathObjnamespace rp {class SingleCellSwathsOrderBase {  + {abstract} genSortedSwaths(const F2CSwaths& swaths,        uint32_t variant = 0) const : F2CSwaths  # {abstract} changeStartPoint(F2CSwaths& swaths,        uint32_t variant) const : void  # {abstract} sortSwaths(F2CSwaths& swaths) const : void = 0}class BoustrophedonOrder {  # sortSwaths(F2CSwaths& swaths) const : void}class SnakeOrder {  # sortSwaths(F2CSwaths& swaths) const : void}class SpiralOrder {  # sortSwaths(F2CSwaths& swaths) const : void}class CustomOrder {  # sortSwaths(F2CSwaths& swaths) const : void}SingleCellSwathsOrderBase <|-- BoustrophedonOrderSingleCellSwathsOrderBase <|-- SnakeOrderSingleCellSwathsOrderBase <|-- SpiralOrderSingleCellSwathsOrderBase <|-- CustomOrder}Swaths <.. rp.SingleCellSwathsOrderBasenamespace sg {class SwathGeneratorBase<typename T> {  + {abstract} generateBestSwaths(f2c::obj::SGObjective& obj,      double op_width, const F2CCell& poly) : F2CSwaths = 0  + {abstract} generateBestSwaths(f2c::obj::SGObjective& obj,      double op_width, const F2CCells& polys) : F2CSwathsByCells  + {abstract} generateSwaths(double angle, double op_width,      const F2CCell& poly) : F2CSwaths  + {abstract} generateSwaths(double angle, double op_width,      const F2CCells& polys) : F2CSwathsByCells}class BruteForce {  + step_angle : double  + generateBestSwaths(f2c::obj::SGObjective& obj,      double op_width, const F2CCell& poly) : F2CSwaths}SwathGeneratorBase <|-- BruteForce}obj.SGObjective  <.. sg.SwathGeneratorBaseSwaths <.. sg.SwathGeneratorBase@endumlTrajectory ToTrajectoryPb(const double header_time,      const std::vector<TrajectoryPoint>& discretized_trajectory)}class Planning {  - std::unique_ptr<Planner> ptr_planner_  - std::vector<TrajectorPoint> last_trajectory_  - double last_header_time_  + bool Plan(const common::vehicle_state::VehicleState& vehicle_state,const bool is_on_auto_mode,        const double publish_time, std::vector<TrajectoryPoint>* discretized_trajectory)  - std::pair<TrajectoryPoint, std::size_t> ComputeStartingPointFromLastTrajectory(const double curr_time) const  - TrajectoryPoint ComputeStartingPointFromVehicleState(        const common::vehicle_state::VehicleState& vehicle_state,const double forward_time) const;  - std::vector<TrajectoryPoint> GetOverheadTrajectory(        const std::size_t matched_index, const std::size_t buffer_size);}note left of Planning  构造函数调用PlannerFactory的静态方法end noteclass PlannerFactory {  + {static} std::unique_ptr<Planner> CreateInstance(                const PlannerType& planner_tyep)}enum PlannerType {  RTK_PLANNER  OTHER}abstract class Planner {  + {abstract} bool Plan(const TrajectoryPoint& start_point,                  std::vector<TrajectoryPoint> *discretized_trajectory) = 0;}class RTKReplayPlanner {  - std::vector<TrajectoryPoint> complete_rtk_trajectory_  + void ReadTrajectoryFile(const std::string& filename)  + bool Plan(const TrajectoryPoint& start_point,        std::vector<TrajectoryPoint> *ptr_trajectory)  - std::size_t QueryPositionMatchedPoint(const TrajectoryPoint& start_point,        const std::vector<TrajectoryPoint>& trajectory) const}class PathPoint {  + double x  + double y  + double z  + double theta  + double kappa  + double s}class TrajectoryPoint {  double relative_time  double v  double a  double dkappa}PathPoint <|-- TrajectoryPointclass ADCTrajectorynote right of ADCTrajectory  最终得到车辆行驶的轨迹end notePlanningNode --> PlanningPlanningNode ..> ADCTrajectory' PlanningNode ..> TrajectoryPointPlanning --> Planner' Planning ..> TrajectoryPointPlannerFactory ..> PlannerType' Planner ..> TrajectoryPointPlanner <|-- RTKReplayPlanner' RTKReplayPlanner --> TrajectoryPointPlanning ..> PlannerFactory@enduml
```

