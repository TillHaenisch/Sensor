defmodule Nbs.Mixfile do
  use Mix.Project

  def project do
    [app: :nbs,
     version: "0.0.1",
     elixir: "~> 1.0",
     escript: escript,
escript_embed_elixir: false, escript_emu_flags: "%%! -noinput\n",
     deps: deps]
  end

def escript do
[ main_module: Nbs, embeded_elixir: true ]
end

  # Configuration for the OTP application
  #
  # Type `mix help compile.app` for more information
  def application do
    [applications: [:logger]]
  end

  # Dependencies can be Hex packages:
  #
  #   {:mydep, "~> 0.3.0"}
  #
  # Or git/path repositories:
  #
  #   {:mydep, git: "https://github.com/elixir-lang/mydep.git", tag: "0.1.0"}
  #
  # Type `mix help deps` for more examples and options
  defp deps do
    []
  end
end
