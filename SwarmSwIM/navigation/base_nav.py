from abc import ABC, abstractmethod


class BaseNavFilter(ABC):
    """
    Abstract distributed navigation-filter interface for SwarmSwIM.

    Design goals
    ------------
    - One filter instance per vehicle
    - Estimator-agnostic base class
    - Minimal coupling with MAC implementation
    - Compatible with local and cooperative navigation
    - Compatible with delayed/asynchronous acoustic updates

    Notes
    -----
    Concrete implementations are expected to:
    - allocate one internal filter state per registered agent
    - implement prediction using the chosen vehicle/process model
    - implement local sensor updates
    - implement cooperative updates from delivered acoustic packets

    Expected simulator flow
    -----------------------
        events = S.tick()
        delivered = MAC(S)
        Ranging(S, delivered)
        Nav(S, delivered)

    Public API
    ----------
    register_agents(agents)
        Create and initialize one internal navigation filter per agent.

    __call__(sim, delivered)
        Perform one navigation step for all agents:
        1) prediction
        2) local updates
        3) cooperative updates

    get_state(agent_or_name)
        Return the internal navigation state object for one agent.

    get_states()
        Return the full dictionary of internal navigation states.
    """

    def __init__(self):
        # per-agent internal filter objects, keyed by agent.name
        self.filters = {}

        # convenience registry of agent objects, keyed by agent.name
        self.agents = {}

    # ==========================================================
    # Public API
    # ==========================================================

    def register_agents(self, agents):
        """
        Register a set of agents and initialize one navigation state per agent.

        Parameters
        ----------
        agents : iterable
            Iterable of SwarmSwIM agent objects. Each agent must expose
            a unique `.name` attribute.
        """
        for agent in agents:
            self.agents[agent.name] = agent
            self.filters[agent.name] = self._init_filter(agent)

    def get_state(self, agent_or_name):
        """
        Return the internal navigation state for one agent.

        Parameters
        ----------
        agent_or_name : str or agent-like
            Agent name, or agent object exposing `.name`.

        Returns
        -------
        Any
            The concrete navigation-state object created by the subclass.
        """
        name = agent_or_name if isinstance(agent_or_name, str) else agent_or_name.name
        return self.filters[name]

    def get_states(self):
        """
        Return all internal navigation states.

        Returns
        -------
        dict
            Dictionary keyed by agent name.
        """
        return self.filters

    def __call__(self, sim, delivered):
        """
        Perform one full navigation step.

        Default sequence:
        1. predict all agents
        2. apply local updates to all agents
        3. process cooperative acoustic updates

        Parameters
        ----------
        sim : object
            Simulation handle.
        delivered : dict
            Delivered acoustic messages as returned by the MAC layer.

        Returns
        -------
        dict
            Internal navigation states dictionary.
        """
        for agent in sim.agents.values():
            self.predict(agent, sim)

        for agent in sim.agents.values():
            self.update_local(agent, sim)

        #
        self.process_cooperative(sim, delivered)

        self.post_step(sim, delivered)

        return self.filters

    # ==========================================================
    # Required subclass hooks
    # ==========================================================

    @abstractmethod
    def _init_filter(self, agent):
        """
        Create and return the concrete internal filter object for one agent.
        """
        raise NotImplementedError

    @abstractmethod
    def predict(self, agent, sim):
        """
        Propagate the navigation state of one agent forward in time.
        """
        raise NotImplementedError

    @abstractmethod
    def update_local(self, agent, sim):
        """
        Apply local onboard measurement updates for one agent.
        """
        raise NotImplementedError

    @abstractmethod
    def process_cooperative(self, sim, delivered):
        """
        Process cooperative acoustic updates using delivered messages.

        Typical inputs include:
        - delivered packet payloads
        - agent.AcousticRange measurements
        """
        raise NotImplementedError

    # ==========================================================
    # Optional subclass hook
    # ==========================================================

    def post_step(self, sim, delivered):
        """
        Optional hook executed after prediction/local/cooperative updates.

        Can be used by subclasses to:
        - export estimates back to agent attributes
        - compute quality metrics
        - maintain histories
        - prepare data for adaptive MAC scheduling
        """
        pass