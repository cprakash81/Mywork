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
      Modelica.Electrical.Analog.Basic.Resistor resistor
        annotation (Placement(transformation(extent={{-58,0},{-38,20}})));
      Modelica.Electrical.Analog.Basic.Inductor inductor
        annotation (Placement(transformation(extent={{-12,0},{8,20}})));
      Modelica.Electrical.Analog.Basic.EMF emf
        annotation (Placement(transformation(extent={{-2,-40},{18,-20}})));
      Modelica.Electrical.Analog.Sources.SignalVoltage signalVoltage
        annotation (Placement(transformation(
            extent={{10,-10},{-10,10}},
            rotation=90,
            origin={-64,-28})));
      Modelica.Mechanics.Rotational.Components.Inertia inertia
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
      Modelica.Blocks.Continuous.PID PID
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
  annotation (uses(Modelica(version="3.2.2")));
end FM3217_2018;
