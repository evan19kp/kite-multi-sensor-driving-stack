See AGENTS.md

<!-- Graphify: uncomment once the codebase is over ~10k lines and the graph is built.

## graphify

This project has a code knowledge graph (classes, functions, imports, calls, communities).
It lives only in the main checkout: C:/REPOS/autonomous-waymo-sim/kite-multi-sensor-driving-stack/graphify-out/graph.json.
Worktrees don't have their own copy, so always pass that path with --graph.

Rules:
- Before your first grep, search or source-file read for a codebase question, run
  `graphify query "<question>" --graph C:/REPOS/autonomous-waymo-sim/kite-multi-sensor-driving-stack/graphify-out/graph.json`.
  Then read raw files only for the lines the graph pointed you to.
- For how two things connect, use `graphify path "<A>" "<B>"`; for one concept, use
  `graphify explain "<concept>"`, with the same --graph path.
- Put these rules in every subagent prompt that explores code.
- The graph reflects main as of the last sprint. Code changed in the current item isn't in it:
  read those files directly.
- Never run `graphify update`, `/graphify`, or any graphify install, hook or uninstall command.
  Evan rebuilds the graph after each sprint.
- If a graphify command fails or the graph file is missing, say so once and continue with
  normal search. Don't retry it.
- Prefer narrow questions: `graphify explain "<ClassOrFunction>"` for a named symbol,
  or a query that names specific classes. Broad questions return mostly tests and CI noise.
- If explain says "Ambiguous", rerun it once with the full node id it lists (the class is
  usually the shorter id). That counts as following the tool's instructions, not a retry.
-->
