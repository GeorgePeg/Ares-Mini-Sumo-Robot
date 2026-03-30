# Ares Mini Sumo Robot
<p>Ares, named after the ancient Greek god of war, refers to the category of Japanese robot wrestling, also known as Sumo. This category branches into several subcategories, such as Mini Sumo. In this repository, there is an improoved version of Ares, created for our participation in Robotic Arena competition, that takes place in Wroclaw, Poland. For more about our team, you can check the follwing GitHub account: (https://github.com/esdaLabWro).</p><br>
<h2><strong>Project Members</strong></h2>
<table class="table table-striped table-bordered" id="project-members">
        <thead class="table-dark">
            <tr>
                <th><strong>Name</strong></th>
                <th><strong>Role in project</strong></th>
                <th><strong>GitHub Account</strong></th>
            </tr>
        </thead>
        <tbody>
            <tr>
                <td>George Pegiazis</td>
                <td>Project Manager</td>
                <td><a href="https://github.com/GeorgePeg">@GeorgePeg</a></td>
            </tr>
            <tr>
                <td>Katerina Tertimpa</td>
                <td>Algorithm Designer</td>
                <td><a href="https://github.com/katerina-kt">@katerina-kt</a></td>
            </tr>
            <tr>
                <td>Ornela-Maria Qoshi</td>
                <td>Circuit Management</td>
                <td><a href="https://github.com/ornela-maria">@ornela-maria</a></td>
            </tr>
        </tbody>
    </table>
<br>
<h4><strong>Special Thanks to:</strong></h4>
<ul>
<li><strong>Fotis Stamatakis</strong> for designing the custom PCB. GitHub Account:<a                         href="https://github.com/FotisRFF">@FotisRFF</a></li>
<li><strong>Basilis Toumpas</strong> for helping during the weld process. GitHub Account:<a href="https://github.com/billtgr">@billtgr</a></li>
</ul>
<br>
<h2><strong>What is a mini sumo robot?</strong></h2>
<p>A mini-sumo robot is a 10x10cm (or 100x100mm) sized version of a sumo robot with a maximum weight range of 0.5kg (or 500gr.).Just like in real-life sumo, the main purpose of a sumo robot is to detect, confront, and push the opponent out
of the circular ring (the dohyo arena) while remaining inside itself. To achieve this, the robot must combine several engineering disciplines:
  <ul>
    <li>precise sensor integration for edge and opponent detection</li>
    <li>fast</li>
    <li>reliable decision-making algorithms and</li>
    <li>efficient motor control to generate the right torque and speed during the fight.</li>
  </ul>
Beyond robotic competition, mini-sumo robots also serve a very important educational purpose. They can be
really helpful for amateurs and students to learn how to think, cooperate, and improve their programming 
skills and their knowledge on electronics, signals & systems, wireless
communication, mechanics, and power management.</p><br>
<h2>PID Simulator</h2>
<p>Here <a href="simulator">🔗</a> you can find a simple PID Simulator we made using Python. We connected the 
.py scripts with the .html file using the Flask framework.</p> To run the simulator you must do the following steps:
<ol>
        <li>Download the code from GitHub.</li>
        <li>Make sure you have Python downloaded on your system. If not, search and download the latest version.</li>
        <li>Open the command window and find the file where the Python has been installed.</li>
        <li>There, you have to run some commands and install Flask using <code>pip</code>.</li> 
        <li>Run <code>pip install pip-upgrade</code> to upgrade <code>pip</code></li>
        <li>To install Flask run the command: <code>pip install Flask</code></li>
        <li>Install <code>numpy</code> using the command: <code>pip install numpy</code></li>
        <li>Now that you've finished with the libraries needed, using the command window find the file you have download the              code from GitHub and there run the following command: <code>python app.py</code> and press Enter.</li>
        <li>After that you will see some messages. Inside those messages, you will notice the link 🔗                         "http://127.0.0.1:5000/". Open your browser and tap <strong>http://127.0.0.1:5000/</strong> and press Enter.</li>
        <li>Finally, you are able to see and interact with the GUI.</li>
</ol>
<p>⚠️ If you run just the .html file you will not be able to interact with the GUI properly.</p>

