within DistrictHeating.Pipes.Examples;
model MSL
  import DistrictHeating;
  extends Modelica.Icons.Example;
  import IDEAS;

  inner Modelica.Fluid.System system(p_ambient=101325)
                                   annotation (Placement(transformation(extent={{60,70},
            {80,90}},          rotation=0)));








  IDEAS.Fluid.Sources.FixedBoundary bou1(
    nPorts=1,
    p=100000,
    T=273.15 + 70,
    redeclare package Medium = IDEAS.Media.Water.Simple)
              annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-86,-52})));
  IDEAS.Fluid.Sources.FixedBoundary bou4(
    use_T=false,
    use_p=true,
    p=100000,
    nPorts=1,
    redeclare package Medium = IDEAS.Media.Water.Simple)
              annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={90,-52})));
  IDEAS.Fluid.Movers.Pump pump1(redeclare package Medium =
        IDEAS.Media.Water.Simple, m_flow_nominal=0.1)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-58,-52})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort senTem1(         m_flow_nominal=0.1,
      redeclare package Medium = IDEAS.Media.Water.Simple)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-30,-52})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort PlugFlowT1(      m_flow_nominal=0.1,
      redeclare package Medium = IDEAS.Media.Water.Simple)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={58,-52})));
  Modelica.Fluid.Pipes.DynamicPipe pipe(
    nNodes=50,
    use_HeatTransfer=true,
    redeclare package Medium = Annex60.Media.Water,
    diameter=0.05,
    length=100,
    redeclare model HeatTransfer =
        Modelica.Fluid.Pipes.BaseClasses.HeatTransfer.ConstantFlowHeatTransfer
        (
        T_ambient=273.15 + 20,
        k=0.026/0.16/2,
        alpha0=0.026/0.16/2))
    annotation (Placement(transformation(extent={{0,-62},{20,-42}})));
equation
  connect(pump1.port_b,senTem1. port_a) annotation (Line(
      points={{-48,-52},{-40,-52}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(pump1.port_a,bou1. ports[1]) annotation (Line(
      points={{-68,-52},{-76,-52}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(PlugFlowT1.port_b, bou4.ports[1]) annotation (Line(
      points={{68,-52},{80,-52}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(pipe.port_b, PlugFlowT1.port_a) annotation (Line(
      points={{20,-52},{48,-52}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(senTem1.port_b, pipe.port_a) annotation (Line(
      points={{-20,-52},{0,-52}},
      color={0,127,255},
      smooth=Smooth.None));
  annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -100},{100,100}}), graphics), Icon(coordinateSystem(extent={{-100,
            -100},{100,100}})),
    experiment(StopTime=2500),
    __Dymola_experimentSetupOutput);
end MSL;
