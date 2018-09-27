within ;
package FM3217_2018 "Collection of models as created in 2018"
  package Tutorial1
    model SimplePendulum " Model of a simple pendulum"

      constant Modelica.SIunits.Acceleration g = 9.81 "Gravitational constant";
      parameter Modelica.SIunits.Length L = 1 "Lenght of the pendulum";

      // Start of the variables

      Modelica.SIunits.Angle Theta(start=0.1, fixed=true);
      Modelica.SIunits.AngularVelocity ThetaDot;

    equation

      ThetaDot = der(Theta);
      der(ThetaDot) = -g/L * sin(Theta);

    end SimplePendulum;
  end Tutorial1;

  package Tutorial2
    model SimplePendulumTut1 " Model of a simple pendulum"

      constant Modelica.SIunits.Acceleration g = 9.81 "Gravitational constant";
      parameter Modelica.SIunits.Length L = 1 "Lenght of the pendulum";

      // Start of the variables

      Modelica.SIunits.Angle Theta(start=0.1, fixed=true);
      Modelica.SIunits.AngularVelocity ThetaDot;

    equation

      ThetaDot = der(Theta);
      der(ThetaDot) = -g/L * sin(Theta);

    end SimplePendulumTut1;

    model Motor
      Modelica.Electrical.Analog.Basic.Ground ground
        annotation (Placement(transformation(extent={{-66,-74},{-46,-54}})));
      Modelica.Electrical.Analog.Basic.Resistor resistor(R=0.5)
        annotation (Placement(transformation(extent={{-58,0},{-38,20}})));
      Modelica.Electrical.Analog.Basic.Inductor inductor(L=0.05)
        annotation (Placement(transformation(extent={{-12,0},{8,20}})));
      Modelica.Electrical.Analog.Basic.EMF emf
        annotation (Placement(transformation(extent={{-2,-40},{18,-20}})));
      Modelica.Electrical.Analog.Sources.SignalVoltage signalVoltage
        annotation (Placement(transformation(
            extent={{10,-10},{-10,10}},
            rotation=90,
            origin={-64,-28})));
      Modelica.Mechanics.Rotational.Components.Inertia inertia(J=0.001)
        annotation (Placement(transformation(extent={{30,-40},{50,-20}})));
      Modelica.Blocks.Interfaces.RealInput u
        annotation (Placement(transformation(extent={{-142,-48},{-102,-8}})));
      Modelica.Mechanics.Rotational.Interfaces.Flange_b flange_b1
                                              "Right flange of shaft"
        annotation (Placement(transformation(extent={{92,-40},{112,-20}})));
    equation
      connect(signalVoltage.p, resistor.p) annotation (Line(points={{-64,-18},{
              -64,10},{-58,10}}, color={0,0,255}));
      connect(resistor.n, inductor.p)
        annotation (Line(points={{-38,10},{-12,10}}, color={0,0,255}));
      connect(inductor.n, emf.p)
        annotation (Line(points={{8,10},{8,-20}}, color={0,0,255}));
      connect(signalVoltage.n, emf.n) annotation (Line(points={{-64,-38},{-64,
              -46},{8,-46},{8,-40}}, color={0,0,255}));
      connect(ground.p, emf.n) annotation (Line(points={{-56,-54},{-56,-46},{8,
              -46},{8,-40}}, color={0,0,255}));
      connect(emf.flange, inertia.flange_a)
        annotation (Line(points={{18,-30},{30,-30}}, color={0,0,0}));
      connect(u, signalVoltage.v)
        annotation (Line(points={{-122,-28},{-71,-28}}, color={0,0,127}));
      connect(inertia.flange_b, flange_b1)
        annotation (Line(points={{50,-30},{102,-30}}, color={0,0,0}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end Motor;

    model Motordrive
      Motor motor
        annotation (Placement(transformation(extent={{8,-6},{28,14}})));
      Modelica.Blocks.Math.Feedback feedback
        annotation (Placement(transformation(extent={{-54,-6},{-34,14}})));
      Modelica.Mechanics.Rotational.Components.Inertia inertia(J=5)
        annotation (Placement(transformation(extent={{66,-8},{86,12}})));
      Modelica.Blocks.Sources.Step step(startTime=0.5)
        annotation (Placement(transformation(extent={{-88,-6},{-68,14}})));
      Modelica.Blocks.Continuous.PID PID(k=10)
        annotation (Placement(transformation(extent={{-26,-6},{-6,14}})));
      Modelica.Mechanics.Rotational.Components.IdealGear idealGear(ratio=100)
        annotation (Placement(transformation(extent={{36,-8},{56,12}})));
      Modelica.Mechanics.Rotational.Sensors.AngleSensor angleSensor annotation (
         Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=-90,
            origin={88,-40})));
    equation
      connect(step.y, feedback.u1)
        annotation (Line(points={{-67,4},{-52,4}}, color={0,0,127}));
      connect(feedback.y, PID.u)
        annotation (Line(points={{-35,4},{-28,4}}, color={0,0,127}));
      connect(motor.u, PID.y) annotation (Line(points={{5.8,1.2},{0.9,1.2},{0.9,
              4},{-5,4}}, color={0,0,127}));
      connect(motor.flange_b1, idealGear.flange_a) annotation (Line(points={{
              28.2,1},{29.1,1},{29.1,2},{36,2}}, color={0,0,0}));
      connect(idealGear.flange_b, inertia.flange_a)
        annotation (Line(points={{56,2},{66,2}}, color={0,0,0}));
      connect(angleSensor.flange, inertia.flange_b)
        annotation (Line(points={{88,-30},{88,2},{86,2}}, color={0,0,0}));
      connect(angleSensor.phi, feedback.u2) annotation (Line(points={{88,-51},{
              22,-51},{22,-48},{-44,-48},{-44,-4}}, color={0,0,127}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end Motordrive;
  end Tutorial2;

  package Tutorial3
    package Components
      model Motor
        Modelica.Electrical.Analog.Basic.Ground ground
          annotation (Placement(transformation(extent={{-66,-74},{-46,-54}})));
        Modelica.Electrical.Analog.Basic.Resistor resistor(R=0.5)
          annotation (Placement(transformation(extent={{-58,0},{-38,20}})));
        Modelica.Electrical.Analog.Basic.Inductor inductor(L=0.05)
          annotation (Placement(transformation(extent={{-12,0},{8,20}})));
        Modelica.Electrical.Analog.Basic.EMF emf
          annotation (Placement(transformation(extent={{-2,-40},{18,-20}})));
        Modelica.Electrical.Analog.Sources.SignalVoltage signalVoltage
          annotation (Placement(transformation(
              extent={{10,-10},{-10,10}},
              rotation=90,
              origin={-64,-28})));
        Modelica.Mechanics.Rotational.Components.Inertia inertia(J=0.001)
          annotation (Placement(transformation(extent={{30,-40},{50,-20}})));
        Modelica.Blocks.Interfaces.RealInput u
          annotation (Placement(transformation(extent={{-142,-48},{-102,-8}})));
        Modelica.Mechanics.Rotational.Interfaces.Flange_b flange_b1
                                                "Right flange of shaft"
          annotation (Placement(transformation(extent={{92,-40},{112,-20}})));
      equation
        connect(signalVoltage.p, resistor.p) annotation (Line(points={{-64,-18},{
                -64,10},{-58,10}}, color={0,0,255}));
        connect(resistor.n, inductor.p)
          annotation (Line(points={{-38,10},{-12,10}}, color={0,0,255}));
        connect(inductor.n, emf.p)
          annotation (Line(points={{8,10},{8,-20}}, color={0,0,255}));
        connect(signalVoltage.n, emf.n) annotation (Line(points={{-64,-38},{-64,
                -46},{8,-46},{8,-40}}, color={0,0,255}));
        connect(ground.p, emf.n) annotation (Line(points={{-56,-54},{-56,-46},{8,
                -46},{8,-40}}, color={0,0,255}));
        connect(emf.flange, inertia.flange_a)
          annotation (Line(points={{18,-30},{30,-30}}, color={0,0,0}));
        connect(u, signalVoltage.v)
          annotation (Line(points={{-122,-28},{-71,-28}}, color={0,0,127}));
        connect(inertia.flange_b, flange_b1)
          annotation (Line(points={{50,-30},{102,-30}}, color={0,0,0}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
              coordinateSystem(preserveAspectRatio=false)));
      end Motor;

      model DCMachine

        parameter Modelica.SIunits.Resistance R = 0.5 " Resistanc of the armature winding";

        Modelica.Electrical.Analog.Basic.Ground ground
          annotation (Placement(transformation(extent={{-66,-74},{-46,-54}})));
        Modelica.Electrical.Analog.Basic.Resistor resistor(R=R)
          annotation (Placement(transformation(extent={{-58,0},{-38,20}})));
        Modelica.Electrical.Analog.Basic.Inductor inductor(L=L)
          annotation (Placement(transformation(extent={{-12,0},{8,20}})));
        Modelica.Electrical.Analog.Basic.EMF emf
          annotation (Placement(transformation(extent={{-2,-40},{18,-20}})));
        Modelica.Mechanics.Rotational.Components.Inertia inertia(J=J)
          annotation (Placement(transformation(extent={{30,-40},{50,-20}})));
        Modelica.Mechanics.Rotational.Interfaces.Flange_b flange_b1
                                                "Right flange of shaft"
          annotation (Placement(transformation(extent={{92,-40},{112,-20}})));
        Modelica.Electrical.Analog.Interfaces.PositivePin p
          "Positive pin (potential p.v > n.v for positive voltage drop v)"
          annotation (Placement(transformation(extent={{-112,26},{-86,52}})));
        Modelica.Electrical.Analog.Interfaces.NegativePin n
          annotation (Placement(transformation(extent={{-112,-52},{-90,-30}})));
        parameter Modelica.SIunits.Inductance L=0.1
          "Inductance of the DC machine";
        parameter Modelica.SIunits.Inertia J=5 "Inertia of the DC Machine"
          annotation (Dialog(tab="Mechanical"));
      equation
        connect(resistor.n, inductor.p)
          annotation (Line(points={{-38,10},{-12,10}}, color={0,0,255}));
        connect(inductor.n, emf.p)
          annotation (Line(points={{8,10},{8,-20}}, color={0,0,255}));
        connect(emf.flange, inertia.flange_a)
          annotation (Line(points={{18,-30},{30,-30}}, color={0,0,0}));
        connect(inertia.flange_b, flange_b1)
          annotation (Line(points={{50,-30},{102,-30}}, color={0,0,0}));
        connect(resistor.p, p) annotation (Line(points={{-58,10},{-80,10},{-80,
                39},{-99,39}}, color={0,0,255}));
        connect(emf.n, n) annotation (Line(points={{8,-40},{-46,-40},{-46,-41},
                {-101,-41}}, color={0,0,255}));
        connect(ground.p, n) annotation (Line(points={{-56,-54},{-56,-41},{-101,
                -41}}, color={0,0,255}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
                Bitmap(
                extent={{96,-61},{-96,61}},
                fileName=
                    "modelica://FM3217_2018/../FM3217/Resources/Images/dc-motor.jpg",
                origin={2,-5},
                rotation=360)}),                                       Diagram(
              coordinateSystem(preserveAspectRatio=false)),
          Documentation(info="<html>
<p><img src=\"modelica://FM3217_2018/Resources/Images/dc-motor.jpg\"/>This is simple DC Machine model.</p>
</html>"));
      end DCMachine;

      model Rload "Resistive Load"
        Modelica.Electrical.Analog.Basic.Resistor resistor(R=R) annotation (
            Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=-90,
              origin={0,10})));
        Modelica.Electrical.Analog.Interfaces.PositivePin p1
          "Positive pin (potential p.v > n.v for positive voltage drop v)"
          annotation (Placement(transformation(extent={{-10,88},{10,108}})));
        Modelica.Electrical.Analog.Interfaces.NegativePin n1
                      "Negative pin"
          annotation (Placement(transformation(extent={{-10,-112},{10,-92}})));
        parameter Modelica.SIunits.Resistance R=0.5 "Load Resistance";
      equation
        connect(resistor.p, p1)
          annotation (Line(points={{0,20},{0,98}}, color={0,0,255}));
        connect(resistor.n, n1)
          annotation (Line(points={{0,0},{0,-102}}, color={0,0,255}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
              coordinateSystem(preserveAspectRatio=false)));
      end Rload;

      model RLLoad
        extends Rload;
        Modelica.Electrical.Analog.Basic.Inductor inductor(L=L_Load)
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=-90,
              origin={-30,10})));
        parameter Modelica.SIunits.Inductance L_Load=0.1 "Inductive Load";
      equation
        connect(inductor.p, p1) annotation (Line(points={{-30,20},{-30,20},{-30,
                40},{-30,40},{-30,42},{0,42},{0,98}}, color={0,0,255}));
        connect(inductor.n, n1) annotation (Line(points={{-30,0},{-28,0},{-28,
                -42},{0,-42},{0,-102}}, color={0,0,255}));
      end RLLoad;

      model RLCLoad
        extends RLLoad;
        Modelica.Electrical.Analog.Basic.Capacitor capacitor(C=C_Load)
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=-90,
              origin={-62,-2})));
        parameter Modelica.SIunits.Capacitance C_Load=0.001 "Capacitance Load";
      equation
        connect(capacitor.p, p1) annotation (Line(points={{-62,8},{-62,8},{-62,
                32},{-62,32},{-62,42},{0,42},{0,98}}, color={0,0,255}));
        connect(capacitor.n, n1) annotation (Line(points={{-62,-12},{-62,-12},{
                -62,-42},{0,-42},{0,-102}}, color={0,0,255}));
      end RLCLoad;

      model Turbine
        Modelica.Mechanics.Rotational.Components.Inertia inertia(J=J_t)
          annotation (Placement(transformation(extent={{-20,-10},{0,10}})));
        Modelica.Mechanics.Rotational.Sources.ConstantTorque constantTorque(
            tau_constant=T_t)
          annotation (Placement(transformation(extent={{76,-8},{56,12}})));
        Modelica.Mechanics.Rotational.Interfaces.Flange_a flange_a annotation (
            Placement(transformation(rotation=0, extent={{-110,-10},{-90,10}})));
        parameter Modelica.SIunits.Inertia J_t "Turbine Inertia";
        parameter Modelica.SIunits.AngularMomentum T_t=10 "Turbine Torque";
      equation
        connect(inertia.flange_b, constantTorque.flange) annotation (Line(
              points={{0,0},{32,0},{32,2},{56,2}}, color={0,0,0}));
        connect(flange_a, inertia.flange_a)
          annotation (Line(points={{-100,0},{-20,0}}, color={0,0,0}));
        annotation (Icon(graphics={Bitmap(extent={{-94,-88},{86,98}}, fileName=
                    "modelica://FM3217_2018/Resources/Images/Turbine.png")}));
      end Turbine;
    end Components;

    package Tests
      model Motordrive
        Tutorial2.Motor motor
          annotation (Placement(transformation(extent={{8,-6},{28,14}})));
        Modelica.Blocks.Math.Feedback feedback
          annotation (Placement(transformation(extent={{-54,-6},{-34,14}})));
        Modelica.Mechanics.Rotational.Components.Inertia inertia(J=5)
          annotation (Placement(transformation(extent={{66,-8},{86,12}})));
        Modelica.Blocks.Sources.Step step(startTime=0.5)
          annotation (Placement(transformation(extent={{-88,-6},{-68,14}})));
        Modelica.Blocks.Continuous.PID PID(k=10)
          annotation (Placement(transformation(extent={{-26,-6},{-6,14}})));
        Modelica.Mechanics.Rotational.Components.IdealGear idealGear(ratio=100)
          annotation (Placement(transformation(extent={{36,-8},{56,12}})));
        Modelica.Mechanics.Rotational.Sensors.AngleSensor angleSensor annotation (
           Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=-90,
              origin={88,-40})));
      equation
        connect(step.y, feedback.u1)
          annotation (Line(points={{-67,4},{-52,4}}, color={0,0,127}));
        connect(feedback.y, PID.u)
          annotation (Line(points={{-35,4},{-28,4}}, color={0,0,127}));
        connect(motor.u, PID.y) annotation (Line(points={{5.8,1.2},{0.9,1.2},{0.9,
                4},{-5,4}}, color={0,0,127}));
        connect(motor.flange_b1, idealGear.flange_a) annotation (Line(points={{
                28.2,1},{29.1,1},{29.1,2},{36,2}}, color={0,0,0}));
        connect(idealGear.flange_b, inertia.flange_a)
          annotation (Line(points={{56,2},{66,2}}, color={0,0,0}));
        connect(angleSensor.flange, inertia.flange_b)
          annotation (Line(points={{88,-30},{88,2},{86,2}}, color={0,0,0}));
        connect(angleSensor.phi, feedback.u2) annotation (Line(points={{88,-51},{
                22,-51},{22,-48},{-44,-48},{-44,-4}}, color={0,0,127}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
              coordinateSystem(preserveAspectRatio=false)));
      end Motordrive;

      model DCMachineTest
        Components.DCMachine dCMachine(R=1)
          annotation (Placement(transformation(extent={{30,-22},{50,28}})));
        Modelica.Electrical.Analog.Sources.SignalVoltage signalVoltage
          annotation (Placement(transformation(
              extent={{24,-24},{-24,24}},
              rotation=90,
              origin={-34,-2})));
        Modelica.Blocks.Sources.Step step
          annotation (Placement(transformation(extent={{-88,-12},{-68,8}})));
        Modelica.Mechanics.Rotational.Components.Inertia inertia
          annotation (Placement(transformation(extent={{64,-8},{84,12}})));
      equation
        connect(signalVoltage.p, dCMachine.p) annotation (Line(points={{-34,22},
                {6,22},{6,12.75},{30.1,12.75}}, color={0,0,255}));
        connect(signalVoltage.n, dCMachine.n) annotation (Line(points={{-34,-26},
                {4,-26},{4,-7.25},{29.9,-7.25}}, color={0,0,255}));
        connect(signalVoltage.v, step.y)
          annotation (Line(points={{-50.8,-2},{-67,-2}}, color={0,0,127}));
        connect(dCMachine.flange_b1, inertia.flange_a) annotation (Line(points=
                {{50.2,-4.5},{58,-4.5},{58,2},{64,2}}, color={0,0,0}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
              coordinateSystem(preserveAspectRatio=false)));
      end DCMachineTest;

      model DCGeneratorTest
        Components.DCMachine dCMachine(R=1)
          annotation (Placement(transformation(extent={{-20,-22},{0,28}})));
        Components.RLCLoad rLCLoad(R=1)
          annotation (Placement(transformation(extent={{-58,-12},{-38,8}})));
        Components.Turbine turbine annotation (Placement(transformation(
                rotation=0, extent={{36,-8},{56,12}})));
      equation
        connect(dCMachine.flange_b1,turbine.flange_a)  annotation (Line(points={{0.2,
                -4.5},{8,-4.5},{8,2},{36,2}},          color={0,0,0}));
        connect(rLCLoad.p1, dCMachine.p) annotation (Line(points={{-48,7.8},{
                -34,7.8},{-34,12.75},{-19.9,12.75}}, color={0,0,255}));
        connect(rLCLoad.n1, dCMachine.n) annotation (Line(points={{-48,-12.2},{
                -34,-12.2},{-34,-7.25},{-20.1,-7.25}}, color={0,0,255}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
              coordinateSystem(preserveAspectRatio=false)));
      end DCGeneratorTest;

      model DCGeneratorextend
        extends DCGeneratorTest(rLCLoad(R=2, C_Load=0.003));
      end DCGeneratorextend;
    end Tests;
  end Tutorial3;

  package Tutorial4
    model ElectricKettle
      Modelica.Electrical.Analog.Basic.Resistor resistor(R=26.45, useHeatPort=
            true) annotation (Placement(transformation(
            extent={{-10,10},{10,-10}},
            rotation=-90,
            origin={0,24})));
      Modelica.Electrical.Analog.Basic.Ground ground
        annotation (Placement(transformation(extent={{-54,-20},{-34,0}})));
      Modelica.Electrical.Analog.Sources.SineVoltage sineVoltage(freqHz=50, V=
            sqrt(2)*230) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=-90,
            origin={-92,26})));
      Modelica.Electrical.Analog.Sensors.PowerSensor powerSensor
        annotation (Placement(transformation(extent={{-46,40},{-26,60}})));
      Modelica.Blocks.Math.Mean mean(f=50) annotation (Placement(transformation(
            extent={{-7,-7},{7,7}},
            rotation=270,
            origin={-51,25})));
      Modelica.Thermal.HeatTransfer.Components.HeatCapacitor water(C=4.18e3*1.7,
          T(fixed=true, start=283.15))
        annotation (Placement(transformation(extent={{16,16},{36,36}})));
      Modelica.Thermal.HeatTransfer.Celsius.TemperatureSensor temperatureSensor
        annotation (Placement(transformation(extent={{44,-26},{64,-6}})));
      Modelica.Electrical.Analog.Ideal.IdealClosingSwitch idealOpeningSwitch
        annotation (Placement(transformation(extent={{-76,40},{-56,60}})));
      Modelica.Thermal.HeatTransfer.Components.ThermalConductor
        thermalConductor(G=5) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=-90,
            origin={26,-36})));
      Modelica.Thermal.HeatTransfer.Celsius.FixedTemperature fixedTemperature(T=
           21) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={28,-68})));
      Modelica.Blocks.Logical.OnOffController onOffController(bandwidth=5)
        annotation (Placement(transformation(extent={{76,-12},{88,0}})));
      Modelica.Blocks.Sources.Constant const(k=95)
        annotation (Placement(transformation(extent={{38,34},{58,54}})));
    equation
      connect(sineVoltage.n, resistor.n) annotation (Line(points={{-92,16},{-92,
              6},{0,6},{0,14}}, color={0,0,255}));
      connect(ground.p, resistor.n) annotation (Line(points={{-44,0},{-44,6},{0,
              6},{0,14}}, color={0,0,255}));
      connect(resistor.p, powerSensor.nc)
        annotation (Line(points={{0,34},{0,50},{-26,50}}, color={0,0,255}));
      connect(powerSensor.nv, resistor.n) annotation (Line(points={{-36,40},{
              -36,6},{0,6},{0,14}}, color={0,0,255}));
      connect(powerSensor.pv, powerSensor.nc) annotation (Line(points={{-36,60},
              {0,60},{0,50},{-26,50}}, color={0,0,255}));
      connect(powerSensor.power, mean.u) annotation (Line(points={{-44,39},{-48,
              39},{-48,33.4},{-51,33.4}}, color={0,0,127}));
      connect(powerSensor.pc, idealOpeningSwitch.n)
        annotation (Line(points={{-46,50},{-56,50}}, color={0,0,255}));
      connect(idealOpeningSwitch.p, sineVoltage.p) annotation (Line(points={{
              -76,50},{-92,50},{-92,36}}, color={0,0,255}));
      connect(water.port, thermalConductor.port_a)
        annotation (Line(points={{26,16},{26,-26}}, color={191,0,0}));
      connect(thermalConductor.port_b, fixedTemperature.port) annotation (Line(
            points={{26,-46},{28,-46},{28,-58}}, color={191,0,0}));
      connect(temperatureSensor.port, thermalConductor.port_a) annotation (Line(
            points={{44,-16},{26,-16},{26,-26}}, color={191,0,0}));
      connect(resistor.heatPort, thermalConductor.port_a) annotation (Line(
            points={{10,24},{10,-2},{26,-2},{26,-26}}, color={191,0,0}));
      connect(temperatureSensor.T, onOffController.u) annotation (Line(points={
              {64,-16},{66,-16},{66,-9.6},{74.8,-9.6}}, color={0,0,127}));
      connect(const.y, onOffController.reference) annotation (Line(points={{59,
              44},{64,44},{64,-2.4},{74.8,-2.4}}, color={0,0,127}));
      connect(onOffController.y, idealOpeningSwitch.control) annotation (Line(
            points={{88.6,-6},{94,-6},{94,82},{-66,82},{-66,57}}, color={255,0,
              255}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end ElectricKettle;
  end Tutorial4;

  model Pipemodel
    HydroPower.HydroSystems.Pipe pipe(horizontalIcon=true, L=100)
      annotation (Placement(transformation(extent={{4,58},{24,78}})));
    inner HydroPower.System_HPL system_HPL(steadyState=true,
        constantTemperature=true)
      annotation (Placement(transformation(extent={{-100,80},{-80,100}})));
    HydroPower.SinksAndSources.Fixed_pT source(paraOption=false)
      annotation (Placement(transformation(extent={{-34,58},{-14,78}})));
    HydroPower.SinksAndSources.Fixed_pT Sink(paraOption=false) annotation (
        Placement(transformation(
          extent={{10,-10},{-10,10}},
          rotation=0,
          origin={56,68})));
  equation
    connect(pipe.b, Sink.b)
      annotation (Line(points={{25,68},{45,68}}, color={0,0,255}));
    connect(pipe.a, source.b)
      annotation (Line(points={{3,68},{-13,68}}, color={0,0,255}));
    annotation (
      Icon(coordinateSystem(preserveAspectRatio=false)),
      Diagram(coordinateSystem(preserveAspectRatio=false)),
      experiment(Tolerance=1e-005, __Dymola_Algorithm="Radau"));
  end Pipemodel;

  model Pipwithvalve
    extends Pipemodel(pipe(ZL=90));
    HydroPower.SinksAndSources.Fixed_pT source1(paraOption=false)
      annotation (Placement(transformation(extent={{-34,14},{-14,34}})));
    HydroPower.SinksAndSources.Fixed_pT Sink1(paraOption=false) annotation (
        Placement(transformation(
          extent={{10,-10},{-10,10}},
          rotation=0,
          origin={58,24})));
    HydroPower.HydroSystems.PipeValve pipeValve1
      annotation (Placement(transformation(extent={{6,12},{26,32}})));
    Modelica.Blocks.Sources.Ramp ramp(
      duration=100,
      offset=0.1,
      startTime=100)
      annotation (Placement(transformation(extent={{-86,26},{-66,46}})));
  equation
    connect(pipeValve1.ValveCtrl, ramp.y) annotation (Line(points={{16,33},{16,
            42},{-65,42},{-65,36}}, color={0,0,127}));
    connect(source1.b, pipeValve1.a) annotation (Line(points={{-13,24},{-4,24},
            {-4,22},{5,22}}, color={0,0,255}));
    connect(Sink1.b, pipeValve1.b) annotation (Line(points={{47,24},{38,24},{38,
            22},{27,22}}, color={0,0,255}));
    annotation (Documentation(info="<html>
<p>Information</p>
<p><br>P = </p>
<p><br>P = 100 MW</p>
<p>H = 90 </p>
<p>Q = </p>
</html>"));
  end Pipwithvalve;
  annotation (uses(Modelica(version="3.2.2"), HydroPower(version="2.7")));
end FM3217_2018;
