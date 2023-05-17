#include <rclcpp/rclcpp.hpp>
//iNCLUYE LAS FUNCIONALIDADES DE ROS2


#include <moveit/planning_scene/planning_scene.h>
//Incluye funcionalidadesc on el robot y con la collision de objetos
#include <moveit/planning_scene_interface/planning_scene_interface.h>


#include <moveit/task_constructor/task.h>
//Incluye los diferentes componentes de MOVEIT TASK CONSTRUCTOR QUE SON USADOS EN EL EJEMPLO

#include <moveit/task_constructor/solvers.h>

#include <moveit/task_constructor/stages.h>

//Incluye diferentes componentes de MoveitTaskConstructor que son usados en el ejemplo
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif

//Se utilizarán para la generación de poses cuando añadamos mas etapas a la tarea
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

//Se usa el objeto logger para registrar informacion relevante durante la ejecución del nodo, y se le pasa el nombre del nodo como argumento
static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_tutorial");
//Se crea un alias de moveit::task_constructor
namespace mtc = moveit::task_constructor;

//Se crea una clase que contiene la funcionalidad del constructor, luego declaramos el objeto de tarea MoveIT Task Constructo como variable miembro
class MTCTaskNode
{
public:
  MTCTaskNode(const rclcpp::NodeOptions& options);//Inicializan el nodo con opciones específicas

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();//esta funcion lo que hace es obtener la interfaz base del nodo, que se usará más adelante.


//Esta funcion primero crea una tarea de moveit Task, luego incluye algunas propiedades y agrega etapas, task.init() inicializa la tarea y task.plan(5), deteniendose después de encontrar 5 planes exitosos, la sig linea publica la solucion que visualizara RVIZ, finalmente task.execution ejecuta el plan
  void doTask();

  void setupPlanningScene();//método para configurar la escena de planificación que se usa en el ejemplo, crea el cilindro y la posicioón, se pueden cambiar los tamaños y eso

private:
  // Compose an MTC task from a series of stages.
  mtc::Task createTask();
  mtc::Task task_;//se crea la variable miembro para nuestra clase, esto no es estrictamente necesario pero ayuda para guardar la tarea para fines posteriores
  rclcpp::Node::SharedPtr node_;
};

//Método de la clase
rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

//Método de la clase
MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("mtc_node", options) }
{
}

//Metodo de la clase
void MTCTaskNode::setupPlanningScene()
{
  moveit_msgs::msg::CollisionObject object;
  object.id = "object";
  object.header.frame_id = "world";
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  object.primitives[0].dimensions = { 0.1, 0.02 };

  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.5;
  pose.position.y = -0.25;
  object.pose = pose;

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(object);
}



//Método de la clase
void MTCTaskNode::doTask()
{
  task_ = createTask();

  try
  {
    task_.init();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return;
  }

  if (!task_.plan(5))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    return;
  }
  task_.introspection().publishSolution(*task_.solutions().front());

  auto result = task_.execute(*task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
    return;
  }

  return;
}


//En este metodo establecemos el nombre de la tarea demo_task, cargamos el modelo de robot, definimos los nombres de marcos utiles, y establecemos esos nombres como propiedades de la tarea con la funcion task.setProperty("",)
mtc::Task MTCTaskNode::createTask()
{
  mtc::Task task;
  task.stages()->setName("demo_task");
  task.loadRobotModel(node_);

  const auto& arm_group_name = "ur_manipulator";
  //const auto& hand_group_name = "hand";
  const auto& hand_frame = "panda_hand";

  // Set task properties
  task.setProperty("group", arm_group_name);
  //task.setProperty("eef", hand_group_name);
  task.setProperty("ik_frame", hand_frame);

// Disable warnings for this line, as it's a variable that's set but not used in this example
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
//Agregamos una etapa de ejemplo al nodo. La primera linea se establece en nullptr, esto crea una etapa de modo que podemos reutilizar la informpacion de la etapa en escenearios específicos.
  mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
#pragma GCC diagnostic pop

 
  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");//Creamos una etapa current state
  
  current_state_ptr = stage_state_current.get();//Esta etapa es la etapa generadora y la agregamos a nuestra tarea. Esto inicia el robot en su estado actual. Ahora que se ha creado el current State se guarda el puntero en el current_state_ptr para usarlo después
  task.add(std::move(stage_state_current));
  std::cout << stage_state_current << std::endl;
  

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);//utiliza el tuberias para la planificaion de moveit
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>(); //Planificador que inerpola entre conjuntos de inicio y meta, se usa para movimientos simples, ya que calcula rápidamente, pero no adminte movimientos complejos

//Establece estrategias específicas para el planificador cartesiano
  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();//Mueve el efector final hacia la pieza en linea recta
  cartesian_planner->setMaxVelocityScaling(1.0);
  cartesian_planner->setMaxAccelerationScaling(1.0);
  cartesian_planner->setStepSize(.01);

//se agregan los planificadores, se crea un escenario que moverá el robot, unas una etapa moveto, dado que abrir la mano es un movimiento relativamente simple, podemos utilizar el planificador de interpolación conjunta. Esta etapa planifica un movimiento a la pose de mano abierta, que es una pose con nombre definidad en el SRDF, para el robot. Devolvemos la tarea y finalizamos con la función create_task
//  auto stage_open_hand =
//      std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
//  stage_open_hand->setGroup(hand_group_name);
//  stage_open_hand->setGoal("open");
//  task.add(std::move(stage_open_hand));

  return task;
}


//En el main se crea un nodo utilizando la clase definida anteriormete y llama a los métodos de clase para configurar y ejecutar una tarea básica de MTC, aqui no se cancela el ejecutor, para inspecciónar las soluciones en rviz
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
    executor.add_node(mtc_task_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mtc_task_node->getNodeBaseInterface());
  });

  mtc_task_node->setupPlanningScene();
  mtc_task_node->doTask();

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}
