# RoboGPT Agentic Pre Execution Workflow

This is the middleware that sits between the web app and the RoboGPT core. It provides functionality such as:
- Prompt checking based on the context.
- Full prompt validation (simulation support).
- Check objects based on the camera view.
- Route requests to RoboGPT core or Simulation.

![Block Diagram](./docs/early_block_diagram.png)
*Early block diagram explaining the complete flow, somethings might have changed now.*

## Installation

I recommend using uv to install the dependencies.

```bash
uv sync
```

To run the agentic workflow, you need to have the RoboGPT core running with vision stack enabled (this agentic workflow needs access to the camera).

```bash
uv run ./main.py
```


## Architecture

This project is build using the OpenAI's Agents SDK and implements various tools, triarge, guardrails for functionality.
Here is quick diagram for understanding the basic workflow:

![Diagram](./docs/architecture.png)

### RoboGPT Interfacing

This agentic workflow accesses tools which are present at RoboGPT core layer over the network via RabbitMQ RPC, the RabbitMQ server is running over AWS using the AmazonMQ service. There are some example scripts in the `tools/backend` directory. The `rpc_server.py` defines the actual functions/tools and is the one who listens to the procedure calls at the RoboGPT core side. The `rpc_client.py` is calling the functions defined in the other script. 

### RoboGPT Tools

There are some tools defined in the RoboGPT Core layer, these tools are called by the agentic workflow. Here is the list of tools that are currently available:

- `check_robot_connection` - Check if the robot is connected to the backend.
- `check_saved_joint` - Check if a pose exists in the robot's memory.
- `check_saved_pose` - Check if a pose exists in the robot's memory.
- `check_object` - Check if an object is detected by the robot's vision system.


For more details, please refer to the [RoboGPT Prompt Validator](https://github.com/orangewood-co/prompt_validator).

### Connection to Frontend

This agentic workflow is connected to the frontend via RabbitMQ RPC queue. The frontend sends the agentic prompt to this queue and the agentic workflow processes the prompt.

```bash
uv run ./prod-new.py
```

Earlier, we were using the `prod.py` script to connect to the frontend. Which uses Pusher to send and receive messages from the frontend.

## References
- OpenAI Agents SDK - [Docs](https://openai.github.io/openai-agents-python/)
- Financial Research Agent Example - [GitHub](https://github.com/openai/openai-agents-python/tree/main/examples/financial_research_agent)
- A Practical guide to building agents - [PDF](https://cdn.openai.com/business-guides-and-resources/a-practical-guide-to-building-agents.pdf)
- RoboGPT Skills Docs - [GitHub](https://github.com/orangewood-co/robogpt_skills_doc)
- RabbitMQ RPC Tutorial - [Docs](https://www.rabbitmq.com/tutorials/tutorial-six-python)