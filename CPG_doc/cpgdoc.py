from reportlab.lib.pagesizes import LETTER
from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer, ListFlowable, ListItem
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.lib import colors

def create_cpg_pdf(filename):
    doc = SimpleDocTemplate(filename, pagesize=LETTER)
    styles = getSampleStyleSheet()
    story = []

    # Custom Styles
    title_style = styles['Title']
    heading_style = styles['Heading2']
    normal_style = styles['Normal']
    code_style = ParagraphStyle(
        'Code',
        parent=styles['Code'],
        fontSize=9,
        leading=12,
        backColor=colors.lightgrey,
        borderPadding=5
    )
    
    # Title
    story.append(Paragraph("CPG (Central Pattern Generator) Guide", title_style))
    story.append(Spacer(1, 12))

    # Intro
    intro_text = """
    This is an excellent and exciting step to take in robotics! Moving from a fixed-state gait (like your run_crawl_gait) to a CPG (Central Pattern Generator) system is a significant upgrade.<br/><br/>
    Your current system is what's called kinematic or trajectory-based. You define a sequence of setpoints (Phase A, Phase B, etc.) and your PID controller moves the motors to those points. You wait for all motors to arrive before starting the next phase.<br/><br/>
    A CPG-based system is dynamic and rhythmic. It doesn't have "steps" or "phases" in the same way. Instead, it uses a network of coupled mathematical oscillators to continuously generate smooth, wave-like setpoints for your motors. The gait "emerges" from the way these oscillators are connected.
    """
    story.append(Paragraph(intro_text, normal_style))
    story.append(Spacer(1, 12))

    # Part 1
    story.append(Paragraph("Part 1: What is a CPG? (The Concept)", heading_style))
    p1_text = """
    Think of the CPG as a "mini-brain" for walking. In animals, CPGs are neural circuits in the spinal cord that produce rhythmic muscle contractions for walking, breathing, or swimming without the brain having to think "now move left-front-leg, now move right-front-leg."<br/><br/>
    For our robot, a CPG is a set of oscillators.<br/><br/>
    <b>Oscillator:</b> An oscillator is just something that produces a regular, repeating signal. The simplest example is a sine wave: output = sin(time). This output goes smoothly up and down, up and down. This is perfect for a leg joint moving back and forth.<br/><br/>
    <b>Central Pattern Generator (CPG):</b> This is a network of oscillators. We'll have one oscillator for each motor (or joint). The key is that they are coupled—they "talk" to each other.<br/><br/>
    By controlling how they talk to each other (the coupling), we can make them synchronize into a pattern. For example, we can "tell" the oscillators for the front-left and back-right legs to move exactly in sync, and tell the other two legs to also move in sync, but exactly 180 degrees out of phase with the first pair. This pattern of synchronization is what creates a trot gait.
    """
    story.append(Paragraph(p1_text, normal_style))
    story.append(Spacer(1, 6))
    
    story.append(Paragraph("<b>Why is this better?</b>", normal_style))
    bullet_list = ListFlowable([
        ListItem(Paragraph("<b>Smooth Motion:</b> The outputs are sine waves, not jerky steps.", normal_style)),
        ListItem(Paragraph("<b>Easy Speed Control:</b> To make the robot walk faster, you just increase the frequency of all the oscillators.", normal_style)),
        ListItem(Paragraph("<b>Easy Gait Transition:</b> To change from a 'trot' to a 'pace,' you just change the coupling rules between the oscillators. The underlying math is the same.", normal_style)),
        ListItem(Paragraph("<b>Robust:</b> The system naturally 'settles' into its pattern, making it more stable.", normal_style))
    ], bulletType='bullet')
    story.append(bullet_list)
    story.append(Spacer(1, 12))

    # Part 2
    story.append(Paragraph("Part 2: The Math (A Simple Oscillator)", heading_style))
    p2_text = """
    We don't need complex differential equations to start. We can use a simple phase oscillator.<br/><br/>
    The most important "state" of an oscillator is its phase, which we'll call phi. This is just an angle that goes from 0 to 2*pi radians (0 to 360 degrees) and then wraps around.<br/><br/>
    <b>Frequency (omega):</b> The speed at which the phase phi changes is its angular frequency, omega. (If you want a frequency f in Hertz (cycles per second), the formula is omega = 2*pi*f).<br/><br/>
    <b>The Update Rule:</b> In a simple loop, the new phase is just the old phase plus a little bit.<br/>
    d_phi/dt = omega (In calculus: "The change in phase over time is the frequency")
    """
    story.append(Paragraph(p2_text, normal_style))
    story.append(Spacer(1, 6))

    story.append(Paragraph("In C Code:", normal_style))
    code_text = """
    phi_new = phi_old + (omega * dt)<br/>
    // We must "wrap" the phase:<br/>
    if (phi_new > 2.0 * M_PI) { phi_new -= 2.0 * M_PI; }
    """
    story.append(Paragraph(code_text, code_style))
    story.append(Spacer(1, 6))

    p2_continued = """
    <b>The Output:</b> The phase phi isn't the motor command. The sine of the phase is. We also need to define the Amplitude (how far the leg moves) and an Offset (the leg's center position).
    """
    story.append(Paragraph(p2_continued, normal_style))
    story.append(Spacer(1, 6))
    story.append(Paragraph("output = Offset + Amplitude * sin(phi)", code_style))
    story.append(Paragraph("This output is the smooth, rhythmic setpoint that we will send to your PID controller.", normal_style))
    story.append(Spacer(1, 12))

    # Part 3
    story.append(Paragraph("Part 3: The 'Network' (Coupling the Oscillators)", heading_style))
    p3_text = """
    This is the magic "G" in CPG. How do we make the oscillators "talk" to each other?<br/>
    We modify the update rule. We add a coupling term that "pulls" or "pushes" an oscillator's phase to match its neighbors.<br/><br/>
    The new update rule for a single oscillator i in a network looks like this:
    """
    story.append(Paragraph(p3_text, normal_style))
    story.append(Spacer(1, 6))
    
    # Equation representation
    equation = "d_phi_i / dt = omega_i + SUM( K_ij * sin(phi_j - phi_i - Delta_ij) )"
    story.append(Paragraph(equation, code_style))
    story.append(Spacer(1, 6))

    p3_details = """
    This looks scary, but it's simple. Let's break it down in C code terms.<br/><br/>
    <b>d_phi[i]:</b> This is the change in phase we will calculate for oscillator i.<br/>
    <b>omega[i]:</b> This is oscillator i's own personal "natural" frequency.<br/>
    <b>sum(...):</b> This means we are going to loop through all other oscillators j.<br/>
    <b>K_ij:</b> This is the Coupling Strength between i and j. A high value means j has a strong influence on i. A value of 0 means they aren't connected.<br/>
    <b>phi[j] - phi[i]:</b> This is the current difference in phase between them.<br/>
    <b>Delta_ij (Delta):</b> This is the Desired Phase Offset. This is the most important part.<br/>
    """
    story.append(Paragraph(p3_details, normal_style))
    story.append(Spacer(1, 6))

    bullet_list_2 = ListFlowable([
        ListItem(Paragraph("If we want i and j to be in-phase (move together), we set Delta_ij = 0.", normal_style)),
        ListItem(Paragraph("If we want i and j to be out-of-phase (opposite), we set Delta_ij = M_PI (180 degrees).", normal_style)),
        ListItem(Paragraph("If we want i to lag j by 90 degrees, we set Delta_ij = -M_PI / 2.0.", normal_style))
    ], bulletType='bullet')
    story.append(bullet_list_2)
    
    p3_end = """
    The sin(...) term is clever: if the current difference (phi[j] - phi[i]) is equal to the desired difference (Delta_ij), the term becomes sin(0), which is 0. No correction is applied! If they are not in sync, this term becomes non-zero and pushes/pulls phi[i] to get it in line.
    """
    story.append(Paragraph(p3_end, normal_style))
    story.append(Spacer(1, 12))

    # Part 4
    story.append(Paragraph("Part 4: Step-by-Step Implementation Guide", heading_style))
    p4_intro = "Let's modify your code. We will:"
    story.append(Paragraph(p4_intro, normal_style))
    
    steps = ListFlowable([
        ListItem(Paragraph("Define a new Oscillator struct.", normal_style)),
        ListItem(Paragraph("Create a global CPG network (an array of these structs).", normal_style)),
        ListItem(Paragraph("Create global 'gait matrices' for K (Coupling) and Delta (Phase Offset).", normal_style)),
        ListItem(Paragraph("Create a new FreeRTOS task, cpg_update_task, to do the math.", normal_style)),
        ListItem(Paragraph("Modify sequence_runner_task to select a gait (by pointing to a gait matrix).", normal_style)),
        ListItem(Paragraph("Critically: Modify your position_loop_task to remove the 'wait for done' logic.", normal_style))
    ], bulletType='bullet')
    story.append(steps)

    # Build PDF
    doc.build(story)

create_cpg_pdf('CPG_Guide.pdf')