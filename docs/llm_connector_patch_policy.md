# LLM Connector Patch Safety Policy

When using the GitHub connector, prefer small, reviewable, low-risk file updates.

Do not use the connector for aggressive, broad, or complicated source rewrites when the change could risk repository integrity, obscure the intended patch, or be difficult to recover from.

If the GitHub connector fails, blocks an update, requires an unsafe workaround, or would require replacing too much code at once, stop using the connector for that code change.

In that case, generate an exact manual patch and ask the user to apply it locally.

Manual patches should be:

```text
- minimal and scoped
- file-by-file
- easy to inspect before applying
- generated as unified diffs or clear replacement hunks
- accompanied by focused build/test commands
```

Do not create branches or pull requests unless explicitly requested.

Do not keep retrying increasingly complex connector paths after a connector failure. Prefer a clean manual patch over a clever tool workaround.

For architecture-sensitive or runtime-sensitive design choices, prepare a concise plan for approval before implementation. Once approved, implement with the least risky mechanism available.

Current 2.26E.11 note:

```text
The header/config surface for follow arrival control was added directly through the connector, and the controller implementation was applied by manual patch in commit ad84ee0.
```
